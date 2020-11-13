/*
 Reads from /tmp/data_exchange, and writes to stdout.
 Redirect this output to a file to save the data_exchange:
    exchange_read /tmp/data_exchange/ * > robot_test3.xcg
*/
#include <memory>
#include "exchange_datatypes.h"
using namespace aurora;

// Read the data exchange T_size header from the file 
size_t read_file_T_size(const char *filename)
{
    FILE *f=fopen(filename,"rb");
    if (!f) return 0;
    data_exchange_disk_header head;
    if (1!=fread(&head,sizeof(head),1,f)) return 0;
    fclose(f);
    return head.T_size;
}

// Read the length of this file 
size_t read_file_size(const char *filename)
{
    FILE *f=fopen(filename,"rb");
    if (!f) return 0;
    fseek(f,0,SEEK_END);
    size_t end=ftell(f);
    fclose(f);
    return end;
}

class exchange_file {
public:
    std::string filename;
    size_t T_size;
    size_t file_size;
private:
    data_exchange_mmap mmap;
    unsigned char *mem;
    
    data_exchange_disk_header last_head;
    
public:
    exchange_file(std::string filename_)
        :filename(filename_),
         T_size(read_file_T_size(filename.c_str())),
         file_size(read_file_size(filename.c_str())),
         mmap(filename.c_str(),file_size,true),
         mem((unsigned char *)mmap.mem)
    {
        last_head.updates=-1; //<- forces one data write at startup
    }
    
    bool updated() {
        return last_head.updates!=
            ((data_exchange_disk_header*)mem)->updates;
    }
    
    void send(FILE *out) {
        last_head=*(data_exchange_disk_header*)mem;
        fwrite(mem,file_size,1,out);
    }
};

int main(int argc,char *argv[]) {
    int millisleep=10; // <- run at 100Hz max
    
    std::vector<std::string> filenames;
    typedef uint64_t filesize_t;
    std::vector<filesize_t> filesizes;
    std::vector<std::unique_ptr<exchange_file> > xchgs;
    //std::vector<exchange_file *> xchgs;
    
    // Each command line argument should be a file to send
    for (int i=1;i<argc;i++) {
        const char *filename=argv[i];
        
        if (0==strcmp(filename,"--slow")) {
            millisleep=2000;
        } 
        else {
            auto xchg=//new exchange_file(filename);
            std::make_unique<exchange_file>(filename);
            if (xchg->T_size==0) {
                fprintf(stderr,"Error reading file size for '%s' (skipping)\n",filename);
            }
            else 
            { // It's a valid file--add to our lists
                fprintf(stderr,"Reading file '%s' file_size %d, T_size %d\n", filename,(int)xchg->file_size,(int)xchg->T_size);
                filesizes.push_back(xchg->file_size);
                filenames.push_back(filename);
                xchgs.push_back(std::move(xchg));
            }
        }
    }
    
    FILE *f=stdout;
    exchange_send(filenames,f);
    exchange_send(filesizes,f);
    fflush(f);
    
    while (1) {
        // Send any changed files
        bool writes=false;
        for (filesize_t i=0;i<xchgs.size();i++)
            if (xchgs[i]->updated()) 
            {
                // Send timestamp (milliseconds UTC)
                millsecond_time_t now=time_in_milliseconds();
                exchange_send(now,f);
                
                // Send file index
                exchange_send(i,f);
                
                // Send file data
                xchgs[i]->send(f);
                writes=true;
            }
        if (writes) fflush(f);
        
        data_exchange_sleep(millisleep);
    }
    
    return 0;
}

