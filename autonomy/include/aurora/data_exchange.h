/*
File-based high performance interprocess data exchange for robotics.
Orion Lawlor, Arsh Chauhan, Addeline Mitchell 2019-11-24
*/
#ifndef __AURORA_DATA_EXCHANGE_H
#define __AURORA_DATA_EXCHANGE_H

#include <stdint.h>
#ifdef _WIN32
#  error "Somebody needs to write a windows version of this header"
#else
#  include <sys/mman.h> // for mmap
#  include <unistd.h> // for seeks
#  include <sys/stat.h> 
#  include <string.h>  // for strerror
#  include <fcntl.h> // for open
#  define DATA_EXCHANGE_DIR "/tmp/data_exchange/"
#  define DATA_EXCHANGE_CHMOD 0777 /* rwx for everybody */
#  define PAGE_SIZE 4096
#endif


namespace aurora {

/** 
 This is the on-disk storage format that we use to exchange data
 of type T, in files in /tmp/data_exchange/.
 
 The type T must be fixed-size plain old data, must be OK with being
 zero initialized, and can't contain any pointers or references.
 To make the files portable between the 32-bit Rasberry Pi and 
 our 64-bit machines, you should use types like int32_t instead of bare int.
*/
template <typename T>
struct data_exchange_ondisk {
public:
    // This is the size, in bytes, of the type T
    //  If this doesn't match, the file and your object
    //  T aren't the same size--you should only use specified-size ints.
    uint64_t T_size;
    
    // This is the data being exchanged
    T data;
    
    // This marks the end of the file
    enum{eof_value=0xe0f};
    uint64_t eof;
};


/**
 Set up a read/write data exchange for data of type T.
 
 You should make one instance of this class early in your program,
 such as a long-lived member, global, or static.
 You shouldn't re-make it repeatedly (such as a local variable).
*/
template <typename T>
class data_exchange {
public:
    // Open this data_exchange
    data_exchange(const std::string &name);
    
    // Do a data consistency check--make sure the file is still OK.
    //   Throws if it finds problems.
    void check(const char *when="");
    
    // Get a read-only copy of the file data
    inline const T &read() const { return mem->data; }
    
    // Get a writeable copy of the file data
    inline T &write() { 
        mem->T_size = sizeof(T);
        mem->eof = data_exchange_ondisk<T>::eof_value;
        return mem->data; 
    }
    
    // Close this data_exchange
    ~data_exchange() {
        if (mem) { munmap(mem,mmap_len); mem=0; mmap_len=0; }
        if (fd) { close(fd); fd=0; }
    }
    
private:
    std::string filename;
    int fd; // UNIX open'd file
    data_exchange_ondisk<T> *mem; // mmap'd file
    size_t mmap_len; // length of file on disk (in pages)
    
    // Don't copy or assign this type
    data_exchange(const data_exchange &e) =delete;
    void operator=(const data_exchange &e) =delete;
};


/*
  This basically just opens and mmaps the file.
  It's complicated because it checks for errors,
  and tries to allow automagic file upgrades,
  without silently breaking things.
*/
template <typename T>
data_exchange<T>::data_exchange(const std::string &name)
    :filename(DATA_EXCHANGE_DIR+name),
     fd(0), mem(0), mmap_len(0)
{
    //   We want rwx for everybody, so that running once as root 
    //   doesn't break the directory for everyone else.
    umask(~DATA_EXCHANGE_CHMOD);
    
    // Try to create the data_exchange directory
    if (0==mkdir(DATA_EXCHANGE_DIR,DATA_EXCHANGE_CHMOD)) 
    { 
        // It didn't already exist.  Print a warning about this.
        printf("Created data exchange directory %s\n",DATA_EXCHANGE_DIR);
    } 
    else if (errno!=EEXIST) 
    { // something actually went wrong making the directory
        throw std::runtime_error(std::string(__FILE__)+" can't create "+DATA_EXCHANGE_DIR+" because "+strerror(errno));
    }
    
    // Open our file (and create it, if it doesn't already exist)
    fd=open(filename.c_str(),O_CREAT|O_RDWR,DATA_EXCHANGE_CHMOD);
    if (fd<0) { 
        throw std::runtime_error(std::string(__FILE__)+" can't create "+filename+" because "+strerror(errno));
    }
    
    // Change the file's length to match our datatype
    mmap_len = sizeof(data_exchange_ondisk<T>);
    
    off_t old_len = lseek(fd,0,SEEK_END);
    if (old_len != 0 && old_len != mmap_len) {
        printf("Upgrading data exchange file %s from %ld byte to %ld byte file length\n",
            filename.c_str(), (long)old_len, (long)mmap_len);
    }
    
    // Make the file be the length we expect (otherwise mmap fails)
    if (0!=ftruncate(fd,mmap_len)) { 
        throw std::runtime_error(std::string(__FILE__)+" can't extend "+filename+" because "+strerror(errno));
    }
    
    // round up to multiple of page size for mmap
    mmap_len = (mmap_len+PAGE_SIZE-1)/PAGE_SIZE*PAGE_SIZE;
    
    // mmap the file
    mem = (data_exchange_ondisk<T> *)mmap(0,mmap_len,
        PROT_READ|PROT_WRITE, MAP_SHARED,
        fd,0);
    if ((long)mem==(long)-1) { 
        throw std::runtime_error(std::string(__FILE__)+" can't mmap "+filename+" because "+strerror(errno));
    }
    
    // Check the file's internal length attribute
    uint64_t old_size=mem->T_size;
    uint64_t new_size=sizeof(T);
    if (old_size != 0 && old_size != new_size) {
        printf("Upgrading data exchange file %s from %ld byte to %ld byte T size\n",
            filename.c_str(), (long)old_len, (long)mmap_len);
    }
    
    // Mark our size.  This will error out any incompatible copies.
    mem->T_size = new_size;
    mem->eof = data_exchange_ondisk<T>::eof_value;
    
    check("Just after opening the file");
}

// Do a data consistency check--make sure the file is still OK.
template <typename T>
void data_exchange<T>::check(const char *when)
{
    bool bad_size = mem->T_size != sizeof(T);
    bool bad_eof = mem->eof != data_exchange_ondisk<T>::eof_value;
    if (bad_size || bad_eof)
    {
        throw std::runtime_error(std::string("Data exchange error: ")+when+" "+__FILE__+" found unexpected data in "+filename+":"+(bad_size?" invalid header size":"")+(bad_eof?" invalid eof marker":"")+".  Is another version running using a different data size?");
    }
}


}; // end namespace aurora

#endif






