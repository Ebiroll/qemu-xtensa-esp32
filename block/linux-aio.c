/*
 * Linux native AIO support.
 *
 * Copyright (C) 2009 IBM, Corp.
 * Copyright (C) 2009 Red Hat, Inc.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "block/aio.h"
#include "qemu/queue.h"
#include "block/block.h"
#include "block/raw-aio.h"
#include "qemu/event_notifier.h"
#include "qemu/coroutine.h"

#include <libaio.h>

/*
 * Queue size (per-device).
 *
 * XXX: eventually we need to communicate this to the guest and/or make it
 *      tunable by the guest.  If we get more outstanding requests at a time
 *      than this we will get EAGAIN from io_submit which is communicated to
 *      the guest as an I/O error.
 */
#define MAX_EVENTS 128

struct qemu_laiocb {
    BlockAIOCB common;
    Coroutine *co;
    LinuxAioState *ctx;
    struct iocb iocb;
    ssize_t ret;
    size_t nbytes;
    QEMUIOVector *qiov;
    bool is_read;
    QSIMPLEQ_ENTRY(qemu_laiocb) next;
};

typedef struct {
    int plugged;
    unsigned int in_queue;
    unsigned int in_flight;
    bool blocked;
    QSIMPLEQ_HEAD(, qemu_laiocb) pending;
} LaioQueue;

struct LinuxAioState {
    AioContext *aio_context;

    io_context_t ctx;
    EventNotifier e;

    /* io queue for submit at batch */
    LaioQueue io_q;

    /* I/O completion processing */
    QEMUBH *completion_bh;
    struct io_event events[MAX_EVENTS];
    int event_idx;
    int event_max;
};

static void ioq_submit(LinuxAioState *s);

static inline ssize_t io_event_ret(struct io_event *ev)
{
    return (ssize_t)(((uint64_t)ev->res2 << 32) | ev->res);
}

/*
 * Completes an AIO request (calls the callback and frees the ACB).
 */
static void qemu_laio_process_completion(struct qemu_laiocb *laiocb)
{
    int ret;

    ret = laiocb->ret;
    if (ret != -ECANCELED) {
        if (ret == laiocb->nbytes) {
            ret = 0;
        } else if (ret >= 0) {
            /* Short reads mean EOF, pad with zeros. */
            if (laiocb->is_read) {
                qemu_iovec_memset(laiocb->qiov, ret, 0,
                    laiocb->qiov->size - ret);
            } else {
                ret = -ENOSPC;
            }
        }
    }

    laiocb->ret = ret;
    if (laiocb->co) {
        qemu_coroutine_enter(laiocb->co);
    } else {
        laiocb->common.cb(laiocb->common.opaque, ret);
        qemu_aio_unref(laiocb);
    }
}

/* The completion BH fetches completed I/O requests and invokes their
 * callbacks.
 *
 * The function is somewhat tricky because it supports nested event loops, for
 * example when a request callback invokes aio_poll().  In order to do this,
 * the completion events array and index are kept in LinuxAioState.  The BH
 * reschedules itself as long as there are completions pending so it will
 * either be called again in a nested event loop or will be called after all
 * events have been completed.  When there are no events left to complete, the
 * BH returns without rescheduling.
 */
static void qemu_laio_completion_bh(void *opaque)
{
    LinuxAioState *s = opaque;

    /* Fetch more completion events when empty */
    if (s->event_idx == s->event_max) {
        do {
            struct timespec ts = { 0 };
            s->event_max = io_getevents(s->ctx, MAX_EVENTS, MAX_EVENTS,
                                        s->events, &ts);
        } while (s->event_max == -EINTR);

        s->event_idx = 0;
        if (s->event_max <= 0) {
            s->event_max = 0;
            return; /* no more events */
        }
        s->io_q.in_flight -= s->event_max;
    }

    /* Reschedule so nested event loops see currently pending completions */
    qemu_bh_schedule(s->completion_bh);

    /* Process completion events */
    while (s->event_idx < s->event_max) {
        struct iocb *iocb = s->events[s->event_idx].obj;
        struct qemu_laiocb *laiocb =
                container_of(iocb, struct qemu_laiocb, iocb);

        laiocb->ret = io_event_ret(&s->events[s->event_idx]);
        s->event_idx++;

        qemu_laio_process_completion(laiocb);
    }

    if (!s->io_q.plugged && !QSIMPLEQ_EMPTY(&s->io_q.pending)) {
        ioq_submit(s);
    }

    qemu_bh_cancel(s->completion_bh);
}

static void qemu_laio_completion_cb(EventNotifier *e)
{
    LinuxAioState *s = container_of(e, LinuxAioState, e);

    if (event_notifier_test_and_clear(&s->e)) {
        qemu_laio_completion_bh(s);
    }
}

static void laio_cancel(BlockAIOCB *blockacb)
{
    struct qemu_laiocb *laiocb = (struct qemu_laiocb *)blockacb;
    struct io_event event;
    int ret;

    if (laiocb->ret != -EINPROGRESS) {
        return;
    }
    ret = io_cancel(laiocb->ctx->ctx, &laiocb->iocb, &event);
    laiocb->ret = -ECANCELED;
    if (ret != 0) {
        /* iocb is not cancelled, cb will be called by the event loop later */
        return;
    }

    laiocb->common.cb(laiocb->common.opaque, laiocb->ret);
}

static const AIOCBInfo laio_aiocb_info = {
    .aiocb_size         = sizeof(struct qemu_laiocb),
    .cancel_async       = laio_cancel,
};

static void ioq_init(LaioQueue *io_q)
{
    QSIMPLEQ_INIT(&io_q->pending);
    io_q->plugged = 0;
    io_q->in_queue = 0;
    io_q->in_flight = 0;
    io_q->blocked = false;
}

static void ioq_submit(LinuxAioState *s)
{
    int ret, len;
    struct qemu_laiocb *aiocb;
    struct iocb *iocbs[MAX_EVENTS];
    QSIMPLEQ_HEAD(, qemu_laiocb) completed;

    do {
        if (s->io_q.in_flight >= MAX_EVENTS) {
            break;
        }
        len = 0;
        QSIMPLEQ_FOREACH(aiocb, &s->io_q.pending, next) {
            iocbs[len++] = &aiocb->iocb;
            if (s->io_q.in_flight + len >= MAX_EVENTS) {
                break;
            }
        }

        ret = io_submit(s->ctx, len, iocbs);
        if (ret == -EAGAIN) {
            break;
        }
        if (ret < 0) {
            /* Fail the first request, retry the rest */
            aiocb = QSIMPLEQ_FIRST(&s->io_q.pending);
            QSIMPLEQ_REMOVE_HEAD(&s->io_q.pending, next);
            s->io_q.in_queue--;
            aiocb->ret = ret;
            qemu_laio_process_completion(aiocb);
            continue;
        }

        s->io_q.in_flight += ret;
        s->io_q.in_queue  -= ret;
        aiocb = container_of(iocbs[ret - 1], struct qemu_laiocb, iocb);
        QSIMPLEQ_SPLIT_AFTER(&s->io_q.pending, aiocb, next, &completed);
    } while (ret == len && !QSIMPLEQ_EMPTY(&s->io_q.pending));
    s->io_q.blocked = (s->io_q.in_queue > 0);
}

void laio_io_plug(BlockDriverState *bs, LinuxAioState *s)
{
    s->io_q.plugged++;
}

void laio_io_unplug(BlockDriverState *bs, LinuxAioState *s)
{
    assert(s->io_q.plugged);
    if (--s->io_q.plugged == 0 &&
        !s->io_q.blocked && !QSIMPLEQ_EMPTY(&s->io_q.pending)) {
        ioq_submit(s);
    }
}

static int laio_do_submit(int fd, struct qemu_laiocb *laiocb, off_t offset,
                          int type)
{
    LinuxAioState *s = laiocb->ctx;
    struct iocb *iocbs = &laiocb->iocb;
    QEMUIOVector *qiov = laiocb->qiov;

    switch (type) {
    case QEMU_AIO_WRITE:
        io_prep_pwritev(iocbs, fd, qiov->iov, qiov->niov, offset);
	break;
    case QEMU_AIO_READ:
        io_prep_preadv(iocbs, fd, qiov->iov, qiov->niov, offset);
	break;
    /* Currently Linux kernel does not support other operations */
    default:
        fprintf(stderr, "%s: invalid AIO request type 0x%x.\n",
                        __func__, type);
        return -EIO;
    }
    io_set_eventfd(&laiocb->iocb, event_notifier_get_fd(&s->e));

    QSIMPLEQ_INSERT_TAIL(&s->io_q.pending, laiocb, next);
    s->io_q.in_queue++;
    if (!s->io_q.blocked &&
        (!s->io_q.plugged ||
         s->io_q.in_flight + s->io_q.in_queue >= MAX_EVENTS)) {
        ioq_submit(s);
    }

    return 0;
}

int coroutine_fn laio_co_submit(BlockDriverState *bs, LinuxAioState *s, int fd,
                                uint64_t offset, QEMUIOVector *qiov, int type)
{
    int ret;
    struct qemu_laiocb laiocb = {
        .co         = qemu_coroutine_self(),
        .nbytes     = qiov->size,
        .ctx        = s,
        .is_read    = (type == QEMU_AIO_READ),
        .qiov       = qiov,
    };

    ret = laio_do_submit(fd, &laiocb, offset, type);
    if (ret < 0) {
        return ret;
    }

    qemu_coroutine_yield();
    return laiocb.ret;
}

BlockAIOCB *laio_submit(BlockDriverState *bs, LinuxAioState *s, int fd,
        int64_t sector_num, QEMUIOVector *qiov, int nb_sectors,
        BlockCompletionFunc *cb, void *opaque, int type)
{
    struct qemu_laiocb *laiocb;
    off_t offset = sector_num * BDRV_SECTOR_SIZE;
    int ret;

    laiocb = qemu_aio_get(&laio_aiocb_info, bs, cb, opaque);
    laiocb->nbytes = nb_sectors * BDRV_SECTOR_SIZE;
    laiocb->ctx = s;
    laiocb->ret = -EINPROGRESS;
    laiocb->is_read = (type == QEMU_AIO_READ);
    laiocb->qiov = qiov;

    ret = laio_do_submit(fd, laiocb, offset, type);
    if (ret < 0) {
        qemu_aio_unref(laiocb);
        return NULL;
    }

    return &laiocb->common;
}

void laio_detach_aio_context(LinuxAioState *s, AioContext *old_context)
{
    aio_set_event_notifier(old_context, &s->e, false, NULL);
    qemu_bh_delete(s->completion_bh);
}

void laio_attach_aio_context(LinuxAioState *s, AioContext *new_context)
{
    s->aio_context = new_context;
    s->completion_bh = aio_bh_new(new_context, qemu_laio_completion_bh, s);
    aio_set_event_notifier(new_context, &s->e, false,
                           qemu_laio_completion_cb);
}

LinuxAioState *laio_init(void)
{
    LinuxAioState *s;

    s = g_malloc0(sizeof(*s));
    if (event_notifier_init(&s->e, false) < 0) {
        goto out_free_state;
    }

    if (io_setup(MAX_EVENTS, &s->ctx) != 0) {
        goto out_close_efd;
    }

    ioq_init(&s->io_q);

    return s;

out_close_efd:
    event_notifier_cleanup(&s->e);
out_free_state:
    g_free(s);
    return NULL;
}

void laio_cleanup(LinuxAioState *s)
{
    event_notifier_cleanup(&s->e);

    if (io_destroy(s->ctx) != 0) {
        fprintf(stderr, "%s: destroy AIO context %p failed\n",
                        __func__, &s->ctx);
    }
    g_free(s);
}
