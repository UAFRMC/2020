/* For aurora data_exchange exchange_read and exchange_write, 
these are the datatypes being exchanged. */
#include <stdio.h>
#include <string>
#include <vector>
#include "aurora/data_exchange.h"

typedef uint64_t exchange_timestamp_t;
typedef uint64_t exchange_size_t;

void fatal(const char *why) { 
    fprintf(stderr,"Fatal error> %s\n",why);
    exit(1);
}

template <class T>
void exchange_send(const T &v,FILE *f) {
	if (1!=fwrite(&v,sizeof(T),1,f)) fatal("error in fwrite T");
}
template <class T>
void exchange_recv(T &v,FILE *f) {
	if (1!=fread(&v,sizeof(T),1,f)) fatal("error in fread T");
}

void exchange_send(const char *bytes,exchange_size_t size,FILE *f) {
	if (size!=fwrite(bytes,1,size,f)) fatal("error in fwrite bytes");
}
void exchange_recv(char *bytes,exchange_size_t size,FILE *f) {
	if (size!=fread(bytes,1,size,f)) fatal("error in fread bytes");
}

void exchange_send(const std::string &v,FILE *f) {
    exchange_size_t size=v.size();
    exchange_send(size,f); // send the size first
    exchange_send(&v[0],size,f); // then the data
}
void exchange_recv(std::string &v,FILE *f) {
    exchange_size_t size=0;
    exchange_recv(size,f); // recv the size first
    v.resize(size);
    exchange_recv(&v[0],size,f); // then the data
}

template <class T>
void exchange_send(const std::vector<T> &v,FILE *f) {
    exchange_size_t size=v.size();
    exchange_send(size,f); // send the size first
    for (exchange_size_t i=0;i<size;i++)
        exchange_send(v[i],f); // then the data
}
template <class T>
void exchange_recv(std::vector<T> &v,FILE *f) {
    exchange_size_t size=v.size();
    exchange_recv(size,f); // send the size first
    v.resize(size);
    for (exchange_size_t i=0;i<size;i++)
        exchange_recv(v[i],f); // then the data
}





