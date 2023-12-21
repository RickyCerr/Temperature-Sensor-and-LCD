// TWI master functions header file: twi_master.h for XMEGA
// Defines and function prototypes for TWI 
// TWI master is set to run under interrupt conditions
// Roger Traylor 6.18.2023

#include <stdbool.h> 

// Buffer size defines  (can be redefined as necessary to save space)
#define TWIM_WR_BUF_SIZE         8
#define TWIM_RD_BUF_SIZE         8

//TWI master struct. Holds pointer to TWI module, buffers and needed variables.
typedef struct twi_master{
	TWI_t   *   module_ptr;                  //pointer to module used 
	register8_t address;                     //slave address 
	register8_t wr_array[TWIM_WR_BUF_SIZE];  //holds data to write 
	register8_t rd_array[TWIM_RD_BUF_SIZE];  //holds data to read
	register8_t bytesToWrite;                //number of bytes to write 
	register8_t bytesToRead;                 //number of bytes to read 
        register8_t bytesWritten;                //number of bytes written 
	register8_t bytesRead;                   //number of bytes read 
	register8_t status;                      //status of transaction 
	register8_t result;                      //result of transaction 
} twi_master_t;

void twi_struct_init(
  twi_master_t      * device_struct, //ptr to a struct, one per device 
  TWI_t             * twi_module,    //ptr to module address, &TWIC or &TWIE 
  uint8_t             address        //slave address
);

void twi_module_init(
         TWI_t           * module, 
       uint8_t             baud_rate,
       TWI_MASTER_INTLVL_t interrupt_lvl); 

bool twi_master_wr_rd(
  twi_master_t * device_struct,
       uint8_t   bytesToWrite,
       uint8_t   bytesToRead);

void twim_wr_handler(twi_master_t * device_struct);

void twim_rd_handler(twi_master_t * device_struct);

void twim_transaction_done(twi_master_t * device_struct, uint8_t result);

void twim_arb_buserr_handler(twi_master_t * device_struct);

twi_master_t  * current_struct; //points to the current structure to use 

//Transaction status defines. 
#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1

//Transaction result enumeration.
typedef enum TWIM_RESULT_enum {
  TWIM_RESULT_UNKNOWN          = (0x00<<0),
  TWIM_RESULT_OK               = (0x01<<0),
  TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
  TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
  TWIM_RESULT_BUS_ERROR        = (0x04<<0),
  TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
  TWIM_RESULT_FAIL             = (0x06<<0),
}TWIM_RESULT_t;

