// twi_master.c  
// R. Traylor
// 6.18.2023
// twi_master code for ATxmega16(32)(64)A4U

//Thanks for sharing the orginal Atmel code on GitHub: 
//https://github.com/jcmcclurg/atmel-atxmega-shared-libraries/blob/master/i2c/twi_master_driver.c
//
//This work leverages the original Atmel code which was a tangled, confusing, incomplete mess.
//Its been refactored and many names have been changed so that their names are
//indicative of their function.
//
//The code here only implements the master TWI functionality.
//
//As required, by open source licensing agreement....
// Copyright (c) 2008, Atmel Corporation All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1. Redistributions of source code must retain the above copyright notice,
// * this list of conditions and the following disclaimer.
// *
// * 2. Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3. The name of ATMEL may not be used to endorse or promote products derived
// * from this software without specific prior written permission.
// *
// THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
// SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// * See Application note: AVR1308: Using the XMEGA TWI

#include <avr/io.h>
#include <stdlib.h>
#include "twi_master.h"
#include <stdbool.h> 
#include <avr/interrupt.h>
  
#define ZERO  0x00
#define ONE   0x01

////////////////////////////////////////////////////////////////////////////////
//                             twi_module_init
//Initalizes modules TWIC or TWIE in master mode. "module_ptr" is a pointer to 
//the address of module used; &TWIC or &TWIE. This allows multiple TWI devices 
//on the same TWI module. All devices on a TWI module must operate with the 
//same baud rate to properly check TWI addresses. 
//
//-> does not initalize any external devices on the selected module.
//-> does not enable the global interrupt.
//-> add pullups on SDA and SCL (~4.7K). Internal pullups are too large (20K)  
//
//Example: twi_module_init(&TWIC,baud_rate,TWI_MASTER_INTLVL_LO_gc);
//
void twi_module_init(
  TWI_t             * twi_module,    //ptr to module address, &TWIC or &TWIE 
  uint8_t             baud_rate,     //TWI bit rate, computed by macro
  TWI_MASTER_INTLVL_t interrupt_lvl) //interrupt level for this module 
{
  twi_module->MASTER.BAUD   = baud_rate;
  //enable rx and tx interrupts, set interrupt level, enable twi_module   
  twi_module->MASTER.CTRLA  = TWI_MASTER_RIEN_bm | 
                              TWI_MASTER_WIEN_bm | 
                              TWI_MASTER_ENABLE_bm | interrupt_lvl; 
  twi_module->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; //force bus state to idle
} //twi_module_init
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                               twi_struct_init
//Initalizes the structure for the device whose pointed to by "device_struct".
//--> does NOT initalize any TWI module registers, only the struct.
//
//Example: twi_struct_init(&TWIC,baud_rate,TWI_MASTER_INTLVL_LO_gc);
//
void twi_struct_init(
  twi_master_t      * device_struct, //ptr to a struct, one per device 
  TWI_t             * twi_module,    //ptr to module address, &TWIC or &TWIE 
  uint8_t             address)       //slave address

{
  device_struct->module_ptr = twi_module; //"module_ptr" is the pointer to the TWI module address 
  device_struct->address   = address;     //device address 
}//twi_struct_init
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                             twi_master_wr_rd
//twi_mstr_wr_rd is called byt main() to either write, read, or write+read data
//between the uC and a TWI peripheral without relinquishing the TWI bus. It 
//looks at the parameters bytesToWrite, bytesToRead. If bytesToRead is zero, it 
//will only write data. If bytesToWrite is zero it will only read. If both  
//parameters are nonzero, it will first write and then read from the peripheral. 
//This function only sends the address (plus R/_W) for the peripheral. If the 
//peripheral responds with an ACK, the ISR will be triggered and will complete 
//the transaction. This function returns a true or false to indicate if the 
//transaction was able to start.
//
//Example: one byte write followed by two bytes:
//return_val=twi_master_wr_rd(current_struct,1,2);
//
bool twi_master_wr_rd(
  twi_master_t * device_struct, //ptr to struct for device
  uint8_t        bytesToWrite,
  uint8_t        bytesToRead) 
{

//removed check for array bounds here (see atmel code)

  uint8_t wr_addr; //temp variable, holds slave address when writing
  uint8_t rd_addr; //temp variable, holds slave address when reading

//start transaction if bus is ready, else return false 
  if(device_struct->status != TWIM_STATUS_READY){return false;}

  device_struct->status = TWIM_STATUS_BUSY;   //rename this to "twi_bus_state"? YES!
  device_struct->result = TWIM_RESULT_UNKNOWN;  //transaction result (master.h)
  device_struct->bytesToWrite = bytesToWrite;
  device_struct->bytesToRead =  bytesToRead;
  device_struct->bytesWritten = 0; 
  device_struct->bytesRead = 0;   
  
  
//set current structure for ISR
current_struct = device_struct; 

//if a write command, send the START condition + Address + 'R/_W = 0' 
  if(device_struct->bytesToWrite > 0){
    wr_addr = device_struct->address & ~0x01;
    device_struct->module_ptr->MASTER.ADDR = wr_addr; //initinate SLA+R/W
  } 
  else{
//if read command, send the START condition + Address + 'R/_W = 1'   
    if(device_struct->bytesToRead > 0){ 
      rd_addr = device_struct->address | 0x01;
      device_struct->module_ptr->MASTER.ADDR = rd_addr;
    }//if device_struct...
  }//else
  return true;
} //masterwriteread
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                                TWI Master Common ISR
//
//Common TWI master interrupt service routine for either PORT C or Port E.
//This ISR is triggered when the device address has been sent out (WIF interrupt)
//or when data bytes are sent or received on the TWI bus. If no errors have 
//occured, the rest of the data is sent or received until none is left. Beyond
//this interrupt, the write and read handler actually does the sending and receiving.
//If an error occurs, the ISR will set the structure result member to "FAIL". The
//This ISR is shared. It works with either port E or C via the ISR alias macro. 
//
ISR(TWIC_TWIM_vect){  //Port C and E aliased below (ISR_ALIAS)   
  uint8_t bus_status; //temp variable for clarity
  bus_status = current_struct->module_ptr->MASTER.STATUS; //get current status
//if we had an multi-master arbitration lost or bus error 
  if((bus_status & TWI_MASTER_ARBLOST_bm) || (bus_status & TWI_MASTER_BUSERR_bm)){
    twim_arb_buserr_handler(current_struct);}
//if a master write interrupt occurred
  else if(bus_status & TWI_MASTER_WIF_bm){twim_wr_handler(current_struct);}
//else if master read interrupt occurred 
  else if(bus_status & TWI_MASTER_RIF_bm){twim_rd_handler(current_struct);}
//if we ended up in an unexpected state, set result as such
  else{   twim_transaction_done(current_struct, TWIM_RESULT_FAIL);}
}//ISR
ISR_ALIAS(TWIE_TWIM_vect,TWIC_TWIM_vect); //same ISR used for port C or E
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                                twim_wr_handler
//
//The TWI master wr handler is called from the master ISR. It sends the actual
//data to the device following the address that was sent successfully. It can
//also send a repeated start if there are bytes to be read. This function does
//not know if its working on the 1st or later bytes in the buffer, so we cannot 
//just use a static variable to keep track of where we are in the buffer. We 
//take care of this by using BytesWritten, etc.
//
void twim_wr_handler(twi_master_t * device_struct){ 

uint8_t data;  //temporary variable for clarity

// If NOT acknowledged (NACK) by slave cancel the transaction. 
  if (device_struct->module_ptr->MASTER.STATUS & TWI_MASTER_RXACK_bm){
    device_struct->module_ptr->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
    device_struct->result                   = TWIM_RESULT_NACK_RECEIVED;
    device_struct->status                   = TWIM_STATUS_READY;
  }
// If more bytes to write, send data. 
  else if ((device_struct->bytesWritten) < (device_struct->bytesToWrite)){
 	 data = device_struct->wr_array[device_struct->bytesWritten];
 	 device_struct->module_ptr->MASTER.DATA = data;
  	 ++(device_struct->bytesWritten); //increment here or let somebody else do it?
  }
// If there are bytes to read, send repeated START condition + Address + R/_W bit = 1
  else if ((device_struct->bytesRead) < (device_struct->bytesToRead)){
    device_struct->module_ptr->MASTER.ADDR = (device_struct->address | 0x01);
  }
// If transaction finished, send STOP condition and set RESULT to OK
  else {
    device_struct->module_ptr->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
    twim_transaction_done(device_struct, TWIM_RESULT_OK);
  }
} //twim_wr_handler
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                            twim_transacton_done
//
//Sets the struct result member value. Marks TWI status as ready. 
//
void twim_transaction_done(
  twi_master_t * device_struct, 
  uint8_t        result)
{
  device_struct->result = result;
  device_struct->status = TWIM_STATUS_READY;
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                             twim_rd_handler
//
//The TWI master read handler moves data from the TWI data register into the
//array for holding data from the TWI bus.
//
void twim_rd_handler(twi_master_t * device_struct){

//Fetch data if bytes are still to be read. 
  if(device_struct->bytesRead < TWIM_RD_BUF_SIZE){ //TODO: Error checking??? Why is this here?
    //now, put data in receive array
    device_struct->rd_array[device_struct->bytesRead] = device_struct->module_ptr->MASTER.DATA;
    device_struct->bytesRead++;
  }
//If TWI rd buffer overflow, issue STOP and BUFFER_OVERFLOW condition. 
  else{
    device_struct->module_ptr->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
    twim_transaction_done(device_struct, TWIM_RESULT_BUFFER_OVERFLOW);
  }
//If more bytes to read, issue ACK and start a byte read. 
  if(device_struct->bytesRead < device_struct->bytesToRead){
  	device_struct->module_ptr->MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
  }
//If transaction finished, issue NACK and STOP condition. 
  else{
    device_struct->module_ptr->MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
    twim_transaction_done(device_struct, TWIM_RESULT_OK);
  }
}//twim_rd_handler
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                           twim_arb_buserr_handler
//
//For a multi-master TWI bus, this function handles both an arbitration lost
//error or in single or multi master systems, a bus error condition. The struct
//member "result" will hold bus status. TODO: check error handler via short circuit
//
void twim_arb_buserr_handler(twi_master_t * device_struct){
  uint8_t bus_status;
  bus_status = device_struct->module_ptr->MASTER.STATUS; //get the bus status
  if (bus_status & TWI_MASTER_BUSERR_bm){                //bus error occurred 
     device_struct->result = TWIM_RESULT_BUS_ERROR;
  }
  else{ 
     device_struct->result = TWIM_RESULT_ARBITRATION_LOST; }//arbitration lost
  //clear which ever interrupt flag was set  TODO: why clearing only one flag???
  device_struct->module_ptr->MASTER.STATUS = bus_status | TWI_MASTER_ARBLOST_bm;
  device_struct->status = TWIM_STATUS_READY; //ready to go again
}
////////////////////////////////////////////////////////////////////////////////

