// //////////////////////////////////////////////////////////
// MemoryMapped.cpp
// Copyright (c) 2013 Stephan Brumme. All rights reserved.
// see http://create.stephan-brumme.com/disclaimer.html
//

#include "MemoryMapped.h"

#include <stdexcept>
#include <cstdio>
#include <stdlib.h>
#include <stddef.h>

// OS-specific
#ifdef __MINGW32__
#define _MSC_VER 1600
//_MSC_VER || 
// Windows
#include <windows.h>
#else
// Linux
// enable large file support on 32 bit systems
#ifndef _LARGEFILE64_SOURCE
#define _LARGEFILE64_SOURCE
#endif
#ifdef  _FILE_OFFSET_BITS
#undef  _FILE_OFFSET_BITS
#endif
#define _FILE_OFFSET_BITS 64
// and include needed headers
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#endif

#ifndef stat64
#define stat64 stat
#endif
#ifndef fstat64
#define fstat64 fstat
#endif

/// do nothing, must use open()
MemoryMapped::MemoryMapped()
: _filename   (),
  _filesize   (0),
  _hint       (Normal),
  _mappedBytes(0),
  _file       (0),
#ifdef _MSC_VER
  _mappedFile (NULL),
#endif
  _mappedView (NULL)
{
}


/// open file, mappedBytes = 0 maps the whole file
MemoryMapped::MemoryMapped(const std::string& filename, size_t mappedBytes, CacheHint hint)
: _filename   (filename),
  _filesize   (0),
  _hint       (hint),
  _mappedBytes(mappedBytes),
  _file       (0),
#ifdef _MSC_VER
  _mappedFile (NULL),
#endif
  _mappedView (NULL)
{
  open(filename, mappedBytes, hint);
}


/// close file (see close() )
MemoryMapped::~MemoryMapped()
{
  close();
}


/// open file
bool MemoryMapped::open(const std::string& filename, size_t mappedBytes, CacheHint hint)
{
  // already open ?
  if (isValid())
    return false;

  _file       = 0;
  _filesize   = 0;
  _hint       = hint;
#ifdef _MSC_VER
  _mappedFile = NULL;
#endif
  _mappedView = NULL;

#ifdef _MSC_VER
  // Windows

  DWORD winHint = 0;
  switch (_hint)
  {
  case Normal:         winHint = FILE_ATTRIBUTE_NORMAL;     break;
  case SequentialScan: winHint = FILE_FLAG_SEQUENTIAL_SCAN; break;
  case RandomAccess:   winHint = FILE_FLAG_RANDOM_ACCESS;   break;
  default: break;
  }

  // open file
  _file = ::CreateFileA(filename.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, winHint, NULL);
  if (!_file)
    return false;

  // file size
  LARGE_INTEGER result;
  if (!GetFileSizeEx(_file, &result))
    return false;
  _filesize = static_cast<uint64_t>(result.QuadPart);

  // convert to mapped mode
  _mappedFile = ::CreateFileMapping(_file, NULL, PAGE_READWRITE , 0, 0, NULL);
  if (!_mappedFile)
    return false;

#else

  // Linux

  // open file , O_RDONLY
  _file = ::open(filename.c_str(), O_RDWR ); // | O_LARGEFILE		 
  if (_file == -1)
  {
    _file = 0;
    return false;
  }

  // file size
  struct stat64 statInfo;
  if (fstat64(_file, &statInfo) < 0)
    return false;

  _filesize = statInfo.st_size;
#endif

  // initial mapping
  remap(0, mappedBytes);

  if (!_mappedView)
    return false;

  // everything's fine
  return true;
}


/// close file
void MemoryMapped::close()
{
  // kill pointer
  if (_mappedView)
  {
#ifdef _MSC_VER
    ::UnmapViewOfFile(_mappedView);
#else
    ::munmap(_mappedView, _filesize);
#endif
    _mappedView = NULL;
  }

#ifdef _MSC_VER
  if (_mappedFile)
  {
    ::CloseHandle(_mappedFile);
    _mappedFile = NULL;
  }
#endif

  // close underlying file
  if (_file)
  {
#ifdef _MSC_VER
    ::CloseHandle(_file);
#else
    ::close(_file);
#endif
    _file = 0;
  }

  _filesize = 0;
}


/// access position, no range checking (faster)
unsigned char MemoryMapped::operator[](size_t offset) const
{
  return ((unsigned char*)_mappedView)[offset];
}


/// access position, including range checking
unsigned char MemoryMapped::at(size_t offset) const
{
  // checks
  if (!_mappedView)
    throw std::invalid_argument("No view mapped");
  if (offset >= _filesize)
    throw std::out_of_range("View is not large enough");

  return operator[](offset);
}


/// raw access
const unsigned char* MemoryMapped::getData() const
{
  return (const unsigned char*)_mappedView;
}


/// true, if file successfully opened
bool MemoryMapped::isValid() const
{
  return _mappedView != NULL;
}


/// get file size
uint64_t MemoryMapped::size() const
{
  return _filesize;
}


/// get number of actually mapped bytes
size_t MemoryMapped::mappedSize() const
{
  return _mappedBytes;
}


/// replace mapping by a new one of the same file, offset MUST be a multiple of the page size
bool MemoryMapped::remap(uint64_t offset, size_t mappedBytes)
{
  if (!_file)
    return false;

  if (mappedBytes == WholeFile)
    mappedBytes = _filesize;

  // close old mapping
  if (_mappedView)
  {
#ifdef _MSC_VER
    ::UnmapViewOfFile(_mappedView);
#else
    ::munmap(_mappedView, _mappedBytes);
#endif
    _mappedView = NULL;
  }

  // don't go further than end of file
  if (offset > _filesize)
    return false;
  if (offset + mappedBytes > _filesize)
    mappedBytes = size_t(_filesize - offset);

#ifdef _MSC_VER
  // Windows

  DWORD offsetLow  = DWORD(offset & 0xFFFFFFFF);
  DWORD offsetHigh = DWORD(offset >> 32);
  _mappedBytes = mappedBytes;

  // get memory address
  _mappedView = ::MapViewOfFile(_mappedFile, (FILE_MAP_WRITE | FILE_MAP_READ) , offsetHigh, offsetLow, mappedBytes);

  if (_mappedView == NULL)
  {
    _mappedBytes = 0;
    _mappedView  = NULL;
    return false;
  }

  return true;

#else

  // Linux
  // new mapping
  _mappedView = ::mmap(NULL, mappedBytes, PROT_READ | PROT_WRITE, MAP_SHARED, _file, offset);
  if (_mappedView == MAP_FAILED)
  {
    _mappedBytes = 0;
    _mappedView  = NULL;
    return false;
  }

  _mappedBytes = mappedBytes;

  // tweak performance
  int linuxHint = 0;
  switch (_hint)
  {
  case Normal:         linuxHint = MADV_NORMAL;     break;
  case SequentialScan: linuxHint = MADV_SEQUENTIAL; break;
  case RandomAccess:   linuxHint = MADV_RANDOM;     break;
  default: break;
  }
  // assume that file will be accessed soon
  //linuxHint |= MADV_WILLNEED;
  // assume that file will be large
  //linuxHint |= MADV_HUGEPAGE;

  ::madvise(_mappedView, _mappedBytes, linuxHint);

  return true;
#endif
}


/// get OS page size (for remap)
int MemoryMapped::getpagesize()
{
#ifdef _MSC_VER
  SYSTEM_INFO sysInfo;
  GetSystemInfo(&sysInfo);
  return sysInfo.dwAllocationGranularity;
#else
  return sysconf(_SC_PAGESIZE); //::getpagesize();
#endif
}

static MemoryMapped* memfile=NULL;

#define FLASH_FILENAME   "esp32flash.bin"

// QEMU, flash emulated f_files
extern "C" const unsigned char* get_flashMemory() 
{

  FILE *file;

 if (memfile!=NULL) {
   return (memfile->getData());
 }

 //fprintf(stderr,"TRYING to MAP esp32flash.bin");


  int X = 4 * 1024 * 1024 ;
  file = fopen(FLASH_FILENAME, "rw+");
  if (file==NULL) {
     file = fopen(FLASH_FILENAME, "wb");    
  }
  if (file==NULL) {
    fprintf(stderr,"esp32flash.bin missing\n");
    exit(1);
  }
  // Posix only
  #ifdef _MSC_VER  
  #else
  if (ftruncate(fileno(file), X)!=0) {
    fprintf(stderr,"esp32flash.bin unable to truncate\n");
  }
  #endif
  //fseek(fp, X , SEEK_SET);
  fclose(file);

  memfile=new MemoryMapped(FLASH_FILENAME);

  if (memfile!=NULL) {
   //fprintf(stderr,"MAPPED esp32flash.bin");
   // unsigned int *data=(unsigned int *)memfile->getData();
   // fprintf(stderr, "esp32flash.bin mapped @ %08X\n", data);

   /* Debug
   for(int j=0;j<1024;j++) 
   {
      printf( "%08X", *data);
      data++;
      if (j%8==7) {
        printf("\n");
      }
   }
  */
   return (memfile->getData());
  }

  return(NULL);
}

