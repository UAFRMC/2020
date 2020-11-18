/*
 Reads from this file (or stdin), and writes files to /tmp/data_exchange.
*/
#include "exchange_datatypes.h"
using namespace aurora;

int main(int argc,char *argv[]) {
    FILE *f=stdin;
    if (argc>1) {
        const char *filename=argv[1];
        f=fopen(filename,"rb");
        if (f==NULL) {
            fprintf(stderr,"Can't open input file '%s'\n",filename);
            return 1;
        }
    }
    
    std::vector<std::string> filenames;
    typedef uint64_t filesize_t;
    std::vector<uint64_t> filesizes;
    
    exchange_recv(filenames,f);
    exchange_recv(filesizes,f);
    
    std::vector<FILE *> files;
    filesize_t max_file=0;
    for (size_t i=0;i<filenames.size();i++)
    {
        const char *filename=filenames[i].c_str();
        printf("Writing filename: '%s' (file_size %d)\n",
            filename,(int)filesizes[i]);
        FILE *file=fopen(filenames[i].c_str(),"rb+");
        if (file==NULL) {
            // Might just need to create it:
            file=fopen(filenames[i].c_str(),"wb");
            if (file==NULL) {
                fprintf(stderr,"ERROR OPENING FILE '%s'\n",filename);
                exit(1);
            }
        }
        files.push_back(file);
        
        max_file=std::max(max_file,filesizes[i]);
    }
    
    char *iobuffer=new char[max_file];
    
    millsecond_time_t real_start=0;
    millsecond_time_t replay_start=0;
    
    while (!feof(f)) {
        // Read a timestamp
        millsecond_time_t replay;
        if (0==fread(&replay,sizeof(replay),1,f)) break; // probably eof
        if (replay_start==0) replay_start=replay;
        replay-=replay_start;
        
        // Get the file index
        filesize_t fileindex=0;
        exchange_recv(fileindex,f);
        if (fileindex<0 || fileindex>filesizes.size()) {
            fprintf(stderr,"Input file index %d failed sanity check!\n",
                (int)fileindex);
        }
        
        // Wait until timestamp arrives
        millsecond_time_t now=time_in_milliseconds();
        if (real_start==0) real_start=now;
        now-=real_start;
        int64_t wait_time=replay-now;
        printf("%30s updated at %d ms\n",
            filenames[fileindex].c_str()+19,(int)replay);
        if (wait_time>0)
            data_exchange_sleep(wait_time);
        
        
        // Get the file data
        filesize_t sz=filesizes[fileindex];
        
        if (1!=fread(iobuffer,sz,1,f)) {
            fprintf(stderr,"Input file ended incomplete (%d remaining)\n",(int)sz);
            return 1;
        }
        
        // Sanity check: iobuffer should end with an eof in the disk footer
        data_exchange_disk_footer *foot=(data_exchange_disk_footer *)
                &iobuffer[sz-sizeof(data_exchange_disk_footer)];
        if (foot->eof!=0 && foot->eof!=data_exchange_disk_footer::eof_value) {
            fprintf(stderr,"Sanity check failed: file footer invalid %08x\n",
                (int)foot->eof);
            return 1;
        }
        
        // Write data to the output
        FILE *out=files[fileindex];
        fseek(out,0,SEEK_SET);
        fwrite(iobuffer,sz,1,out);
        fflush(out);
    }
    
    return 0;
}

