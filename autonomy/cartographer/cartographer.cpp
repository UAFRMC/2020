#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"


int main(){
    MAKE_exchange_field_drivable();
    MAKE_exchange_field_raw();

    aurora::field_drivable newField = exchange_field_raw.read();

    while(true){
        if(exchange_field_raw.updated()){
            newField = exchange_field_raw.read();
        }
        
        exchange_field_drivable.write_begin() = newField;
        exchange_field_drivable.write_end();
    }
    return 0;
}