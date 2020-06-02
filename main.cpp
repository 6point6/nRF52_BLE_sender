/*
    Structure based on https://github.com/ARMmbed/mbed-os-example-ble/blob/master/BLE_LEDBlinker/source/main.cpp


 */

// Platform Libs
#include <events/mbed_events.h>
#include <mbed.h>

// BLE libs
#include "ble/BLE.h"
#include "ble/gap/AdvertisingDataParser.h"

// C++ libs
#include <cstring>
#include <sstream>
#include <map>

// Protobuf
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "protobuf/adv_packet.pb.h"

// Constants
#define BLINKING_RATE_MS        500               // LED blinking rate in milliseconds
#define PC_SERIAL_BAUD          115200            // Serial baud rate

/*
    Active scanning sends scan requests to elicit scan responses
 */
#define ACTIVE_SCANNING         true              

/*
    Scan interval is the time it waits on a single advertising channel
 */
#define SCAN_INTERVAL           1000              

/*
    From the API documentation:
    The scanning window divided by the interval determines the duty cycle for scanning. For example, if the interval is 100ms and the window is 10ms, then the controller will scan for 10 percent of the time. It is possible to have the interval and window set to the same value. In this case, scanning is continuous, with a change of scanning frequency once every interval.
 */
#define SCAN_WINDOW             1000


#define ADDR_STRING_LEN         12      // chars in hex string address

#define DEBUG                   true

// Global Objects
static EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);
Serial _pc_serial(USBTX, USBRX);            // define the Serial object


/*
    Inner scanner class
 */
class BLEScanner : ble::Gap::EventHandler {
public:
    // constructor
    BLEScanner(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _alive_led(LED1, 1) { }

    // destructor
    ~BLEScanner() { }

    // scan method 
    void scan() {
        // sets the instance of the scanner class as the event handler
        _ble.gap().setEventHandler(this);

        // Initialise the BLE stack, start scanning if successful
        _ble.init(this, &BLEScanner::on_init_complete);

        // add the LED blinker as a recurring event on the event queue
        _event_queue.call_every(BLINKING_RATE_MS, this, &BLEScanner::blink);

        // run infinitely
        _event_queue.dispatch_forever();
    }

// Private scanner class variables and methods
private:
    // private global scanner variables
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _alive_led;

    // print our address
    void print_local_address()
    {
        /* show what address we are using now */
        Gap::AddressType_t addr_type;
        Gap::Address_t address;
        _ble.gap().getAddress(&addr_type, address);
        printf("Device address: ");
        _pc_serial.printf("Local address: %02x:%02x:%02x:%02x:%02x:%02x.\r\n",
            address[5], address[4], address[3], address[2], address[1], address[0]);
    }
  

    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        
        // report error or success
        if (params->error != BLE_ERROR_NONE) {
            _pc_serial.printf("Ble initialisation failed.\r\n");
            return;
        }

        _pc_serial.printf("Ble initialisation complete.\r\n");

        // print local address
        print_local_address();

        // setup scan with custom parameters
        ble::ScanParameters scan_params;
        scan_params.set1mPhyConfiguration(ble::scan_interval_t(SCAN_INTERVAL), ble::scan_window_t(SCAN_WINDOW), ACTIVE_SCANNING);
        _ble.gap().setScanParameters(scan_params);
        
        // start scanning
        _ble.gap().startScan();
    }

    // Blink the alive LED
    void blink() {
        _alive_led = !_alive_led;
    }
       
    
    // Called on receipt of an advertising report
    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        
        // get the address and RSSI from the event
        const ble::address_t address = event.getPeerAddress();
        const ble::rssi_t rssi = event.getRssi();
        ble::AdvertisingDataParser adv_data(event.getPayload());

        if(DEBUG) {
            _pc_serial.printf("Packet received from %02x:%02x:%02x:%02x:%02x:%02x.\r\n",
            address[5], address[4], address[3], address[2], address[1], address[0]);
        }

        // prep pb object and data buffer
        BLE_adv_packet BLE_adv_packet = BLE_adv_packet_init_zero;
        uint8_t buffer[128];

        // attach the buffer to the output stream
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        
        // populate it with our data
        BLE_adv_packet.rssi = rssi;
        BLE_adv_packet.dataLen = event.getPayload().size();

        // copy the address bytes
        for(int i = 0; i < Gap::ADDR_LEN; i++) {
            BLE_adv_packet.address[i] = address[Gap::ADDR_LEN - 1 - i];
        }

        // encode the data, report on the status
        bool status = pb_encode(&stream, BLE_adv_packet_fields, &BLE_adv_packet);

        if (!status)
        {
            _pc_serial.printf("Failed to encode packet data\r\n");
        } else {
            if(DEBUG)
                _pc_serial.printf("Encoded fine.\r\n");
        }

        // print the encoded data to serial, with the length
        if(DEBUG)
            _pc_serial.printf("Message Length: %d.\r\nMessage: ", stream.bytes_written);

        // send it over serial
        for(int i = 0; i < stream.bytes_written; i++){
            _pc_serial.printf("%02x", buffer[i]);
        }

        _pc_serial.printf("\r\n");   
    }    
}; /*  /Inner scanner class */


/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

// Main method sets up peripherals, runs infinite loop
int main()
{   
    // setup serial connection
    _pc_serial.baud(PC_SERIAL_BAUD);

    // create BLE instance
    BLE &ble = BLE::Instance();

    // attach the callback to the event queue
    ble.onEventsToProcess(schedule_ble_events);

    // Setup scanner instance
    BLEScanner scanner(ble, event_queue);

    // Run the scanner
    _pc_serial.printf("Setup done. Running.\r\n");
    scanner.scan();
 
    return 0;
}