#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"


int main(){
    MAKE_exchange_field_drivable();
    MAKE_exchange_field_raw();

    unsigned char mark=aurora::field_unknown;
    aurora::field_drivable newField;
    newField.clear(mark);
    exchange_field_drivable.write_begin() = newField;
    exchange_field_drivable.write_end();
    while(true){
        if(exchange_field_raw.updated()){
            newField = exchange_field_raw.read();
        }

        exchange_field_drivable.write_begin() = newField;
        exchange_field_drivable.write_end();
    }
    return 0;
}