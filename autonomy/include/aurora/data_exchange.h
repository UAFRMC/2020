/*
File-based high performance interprocess data exchange for robotics.
Orion Lawlor, Arsh Chauhan, Addeline Mitchell 2019-11-24
*/
#ifndef __AURORA_DATA_EXCHANGE_H
#define __AURORA_DATA_EXCHANGE_H

#include <chrono> // for timers
#include <stdint.h> // for uint64_t and such
#include <atomic> // for std::atomic_thread_fence
#include <stdexcept> // for std::runtime_error
#include <string.h>  // for strerror

#ifdef _WIN32
#  error "Somebody needs to write a windows version of this header"
#else
#  include <sys/mman.h> // for mmap
#  include <unistd.h> // for seeks
#  include <sys/stat.h> 
#  include <fcntl.h> // for open
#  define PAGE_SIZE 4096
#endif

#  define DATA_EXCHANGE_DIR "/tmp/data_exchange/"
#  define DATA_EXCHANGE_CHMOD 0777 /* rwx for everybody */

namespace aurora {



inline void make_data_exchange_dir(bool silent=false) {
    // Try to create the data_exchange directory
    if (0==mkdir(DATA_EXCHANGE_DIR,DATA_EXCHANGE_CHMOD)) 
    { 
        // It didn't already exist.  Print a warning about this.
        if (!silent) printf("Created data exchange directory %s\n",DATA_EXCHANGE_DIR);
    } 
    else if (errno!=EEXIST) 
    { // something actually went wrong making the directory
        throw std::runtime_error(std::string(__FILE__)+" can't create "+DATA_EXCHANGE_DIR+" because "+strerror(errno));
    }
}

// This is an opened file, mapped read/write into memory.
//  This is shared by the templated class below, and runtime classes in exchange_read.
class data_exchange_mmap
{
    int fd; // UNIX open'd file
public:
    void *mem; // mapped memory
    size_t mmap_len; // length of file on disk (in pages)
    
    /*
     Open a data_exchange file for mmap.  Return the mmap'd memory.
    */
    data_exchange_mmap(const char *filename,size_t bytelength,bool silent=false)
        :fd(0), mem(0), mmap_len(bytelength)
    {
        // Can't mmap a zero-length file
        if (bytelength==0) return; 
        
        //   We want rwx for everybody, so that running once as root 
        //   doesn't break the directory for everyone else.
        mode_t old_mask=umask(~DATA_EXCHANGE_CHMOD);
        
        make_data_exchange_dir();
        
        // Warn if we're creating a new file
        if (0!=access(filename,F_OK)) {
            if (!silent) printf("Creating new exchange file %s\n",filename);
        }
        
        // Open our file (and create it, if it doesn't already exist)
        fd=open(filename,O_CREAT|O_RDWR,DATA_EXCHANGE_CHMOD);
        if (fd<0) { 
            throw std::runtime_error(std::string(__FILE__)+" can't create "+filename+" because "+strerror(errno));
        }
    
        umask(old_mask);
        
        uint64_t old_len = lseek(fd,0,SEEK_END);
        if (old_len != 0 && old_len != mmap_len) {
            if (!silent) printf("Upgrading data exchange file %s from %ld byte to %ld byte file length\n",
                filename, (long)old_len, (long)mmap_len);
        }
        
        // Make the file be the length we expect (otherwise mmap fails)
        //  SUBTLE: for a new file, this also zeros out the file data.
        if (0!=ftruncate(fd,mmap_len)) { 
            throw std::runtime_error(std::string(__FILE__)+" can't extend "+filename+" because "+strerror(errno));
        }
        
        // round up to multiple of page size for mmap
        mmap_len = (mmap_len+PAGE_SIZE-1)/PAGE_SIZE*PAGE_SIZE;
        
        // mmap the file
        mem = mmap(0,mmap_len,
            PROT_READ|PROT_WRITE, MAP_SHARED,
            fd,0);
        if ((long)mem==(long)-1) { 
            throw std::runtime_error(std::string(__FILE__)+" can't mmap "+filename+" because "+strerror(errno));
        }
    }
    
    ~data_exchange_mmap() {
        if (mem) { munmap(mem,mmap_len); mem=0; mmap_len=0; }
        if (fd) { close(fd); fd=0; }
    } 
};



/** This is the on-disk format for the start of a data_exchange file. */
struct data_exchange_disk_header {
    // This is the size, in bytes, of the type T of data being exchanged.
    //  If this doesn't match, the file and your object
    //  T aren't the same size--check for version mismatch.
    uint64_t T_size;
    
    // This counter changes everytime somebody calls write_end().
    //   It's more useful for finding hangs than really tracking changes.
    uint32_t updates;

    // These are flags, like an in-use marker
    uint32_t flags;
    enum {
        flag_being_written=1<<0 // bit 0: a write is in progress now
    };
/*
    // This is an atomic integer, used as a mutex for data writes 
    std::atomic<uint32_t> write_lock;
*/
};

/** This is the on-disk format for the end of a data_exchange file. */
struct data_exchange_disk_footer {
    // This marks the end of the file
    enum{eof_value=0xe0f};
    uint32_t eof;
};

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
    // The header contains the data size
    data_exchange_disk_header header;
    
    // This is the data being exchanged
    T data;
    
    // The footer contains an update counter, and end-of-file marker.
    data_exchange_disk_footer footer;
};


/**
 Set up a read/write data exchange for data of type T.
 
 T MUST be trivially copyable--it can be a struct or class, but it can't contain pointers.
 This means std::array is OK, but std::vector or std::map or std::string won't work.
 
 You should make one instance of this class early in your program,
 such as a long-lived member, global, or static.
 You shouldn't re-make it repeatedly (such as a local variable).
*/
template <typename T>
class data_exchange {
#if !defined(__GNUC__) || __GNUC__>=5 // missing from gcc 4
    // C++11 macro magic to enforce T datatype limits.
    static_assert(std::is_trivially_copyable<T>::value, "Data exchange datatypes are exchanged as raw bytes in files, so they can't contain pointers (like std::vector or std::string), or have copy constructors, move constructors, or destructors.  We use std::is_trivially_copyable to determine this.");
#endif

public:
    // Open this data_exchange
    data_exchange(const std::string &name);
    
    // Do a data consistency check--make sure the file is still OK.
    //   Throws if it finds problems.
    // Returns the current write count.
    uint32_t check(const char *when="");
    
    // Return true if this data has been updated since the last read()
    bool updated(void) const {
        return last_update != mem->header.updates;
    }
    
    // Get a read-only copy of the file data
    inline const T &read() { 
        // Do I want a memory read fence here?  std::atomic_thread_fence();
        last_update = mem->header.updates;
        return mem->data; 
    }
    
    // Get a writeable copy of the file's stored data.
    //   Returns a reference to writeable data.
    inline T &write_begin(void) { 
        mem->header.T_size = sizeof(T); //<- our write is this size
        mem->header.flags |= (uint32_t)(data_exchange_disk_header::flag_being_written);
        return mem->data; 
    }
    
    // Finish a write operation, making these changes visible outside.
    inline void write_end(void) {
        last_update = ++mem->header.updates;
        mem->header.flags &= ~(uint32_t)(data_exchange_disk_header::flag_being_written);
        mem->footer.eof = data_exchange_disk_footer::eof_value;
        // Do I want a write fence here?  std::atomic_thread_fence(std::memory_order_release);
    }
    
    // Close this data_exchange
    ~data_exchange() {
    }
    
private:
    std::string filename;
    data_exchange_mmap mmap;
    data_exchange_ondisk<T> *mem; // mmap'd file
    uint32_t last_update; // last-seen value of updates
    
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
     mmap(filename.c_str(),sizeof(data_exchange_ondisk<T>))
{
    mem = (data_exchange_ondisk<T> *)mmap.mem;
    
    // Check the file's internal length attribute
    uint64_t old_size=mem->header.T_size;
    uint64_t new_size=sizeof(T);
    if (old_size != 0 && old_size != new_size) {
        printf("Upgrading data exchange file %s from %ld byte to %ld byte T size\n",
            filename.c_str(), (long)old_size, (long)new_size);
    }
    
    // Mark our size.  This will error out any incompatible copies.
    mem->header.T_size = new_size;
    mem->footer.eof = data_exchange_disk_footer::eof_value;
    
    check("Just after opening the file");
}

// Do a data consistency check--make sure the file is still OK.
//   Returns the current update count.
template <typename T>
uint32_t data_exchange<T>::check(const char *when)
{
    bool bad_size = mem->header.T_size != sizeof(T);
    bool bad_eof = mem->footer.eof != data_exchange_disk_footer::eof_value;
    if (bad_size || bad_eof)
    {
        throw std::runtime_error(std::string("Data exchange error: ")+when+" "+__FILE__+" found unexpected data in "+filename+":"+(bad_size?" invalid header size":"")+(bad_eof?" invalid eof marker":"")+".  Is another version running using a different data size?");
    }
    return mem->header.updates;
}


// Millisecond timestamps
// Wraps around every half billion years
typedef uint64_t millsecond_time_t;

// Return the real time in milliseconds since the UTC epoch (1970)
millsecond_time_t time_in_milliseconds() {
	auto now=std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>
		(now.time_since_epoch()).count();
}



#define NANO_TO_MILLI 1000000UL
/* Sleep for this many milliseconds.
    1ms sleep -> about 1% CPU used.
    10ms sleep -> under 0.1% CPU used.
*/
void data_exchange_sleep(int millisec=1) {
    // Don't hog the CPU, give up our timeslice
    struct timespec sleeptime;
    sleeptime.tv_sec=millisec/1000;
    sleeptime.tv_nsec=(millisec%1000)*NANO_TO_MILLI;
    nanosleep(&sleeptime,NULL);
}

}; // end namespace aurora

#endif






