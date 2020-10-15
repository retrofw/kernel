#include "hdcp.h"
#include "ep932_if.h"
#include "ddc_if.h"


unsigned int hdcp_timecount = 0;
hdcp_state_t hdcp_state = 0;
unsigned char hdcp_status = 0;
unsigned char ri_check = 0, fake_hdcp = 0;
unsigned char *pksvlist = 0, *pbksv_bcaps3 = 0, *psha_m0 = 0;
unsigned char ksvlistnumber = 0;


//
// Global Data
//
unsigned long temp_sha_h[5];
unsigned char temp_hdcp[10];

//int i, j;

//
// Private Functions
//

unsigned char hdcp_validate_ri(void);

// Repeater
unsigned char hdcp_compute_sha_message_digest(unsigned char hdcp_dev_count, unsigned char hdcp_depth);

// SHA
void sha_initial(void);
void sha_push_data(unsigned char *pdata, unsigned char size);
unsigned long *sha_get_sha_digest(void);
void sha_calculation(unsigned long psha_h[5], unsigned long psha_w1[16]);

//---------------------------------------------------------------------------------------------------------------------------

void hdcp_extract_bksv_bcaps3(unsigned char *bksv_bcaps3)
{
	pbksv_bcaps3 = bksv_bcaps3;
}

void hdcp_extract_fifo(unsigned char *pfifolist, unsigned char listnumber)
{
	pksvlist = pfifolist;
	ksvlistnumber = listnumber;
}

void hdcp_extract_sha_m0(unsigned char *sha_m0)
{
	psha_m0 = sha_m0;
}

void hdcp_stop()
{
	hdcp_timecount = 1000/HDCP_TIMER_PERIOD; // No delay for next startup
	hdcp_status = 0;
	hdcp_state = 0;
	ri_check = 0;

	// Disable the HDCP Engine
	hdmi_tx_hdcp_disable();

	// Disable mute for transmiter video and audio
	hdmi_tx_mute_disable();
}

void hdcp_fake(unsigned char enable)
{
	fake_hdcp = enable;
}

unsigned char hdcp_get_status(void)
{
	return hdcp_status;
}

void hdcp_timer(void) 
{
	++hdcp_timecount;
}

void hdcp_ext_ri_trigger(void) 
{
	ri_check = 1;
}

hdcp_state_t hdcp_authentication_task(unsigned char receiverrdy)
{
	if( (hdcp_state > A0_WAIT_FOR_ACTIVE_RX) && !(receiverrdy) ) {
		DBG_PRINTK(("WARNING: No RSEN or Hot-Plug in Authentication\r\n"));	
			
		// enable mute for transmiter video and audio
		hdmi_tx_mute_enable();

		// Confirm, Disable the HDCP Engine (actived from Downstream)
		hdmi_tx_hdcp_disable();

		// Restart HDCP authentication
		hdcp_timecount = 0;
		hdcp_state = A0_WAIT_FOR_ACTIVE_RX;
		hdcp_status |= HDCP_ERROR_RSEN;
		return hdcp_state;
	}	   

	switch(hdcp_state) {
	// HDCP authentication

		// A0 state -- Wait for Active RX
		//          -- read and validate BKSV for every 1 second.
		case A0_WAIT_FOR_ACTIVE_RX:
 		    
printk("((((((((((((((((((((((((((((((((((((((((A0_WAIT_FOR_ACTIVE_RX:))))))))))))))))))))))))))))))))))))))))\n");
 	    	if(hdcp_timecount > 1000/HDCP_TIMER_PERIOD) {
				hdcp_timecount = 0;
   
				if( downstream_rx_read_bksv(temp_hdcp) ) {		
					int i;
					for (i = 0; i < 10; i++)
						printk("tmp[%d] = %#x\n", i, temp_hdcp[i]);
					DBG_PRINTK(("Authentication start...\r\n"));
		   			hdcp_state = A1_EXCHANGE_KSVS; }
				else {	// HDCP not support
					DBG_PRINTK(("HDCP might not be supported\r\n"));
					downstream_rx_write_ainfo(0x00); // Make TE sense the retry to pass ATC HDCP test
					hdcp_status |= HDCP_ERROR_BKSV;
				}
			}
			break;

		// A1 state -- Exchange KSVs
		//          -- retrieve the random number
		//          -- write AN to RX and TX
		//          -- read and write AKSV, BKSV
		case A1_EXCHANGE_KSVS:

printk("((((((((((((((((((((((((((((((((((((((((A1:))))))))))))))))))))))))))))))))))))))))\n");
			// Write AINFO
			downstream_rx_write_ainfo(0x00);

printk("((((((((((((((((((((((((((((((((((((((((A1:0))))))))))))))))))))))))))))))))))))))))\n");
			// Check Repeater Bit
			temp_hdcp[0] = downstream_rx_bcaps();
			if(temp_hdcp[0] & 0x40) {		// REPEATER
				hdmi_tx_rptr_set();
				printk("repeater.\n");
			}
			else {						// NON-REPEATER
				hdmi_tx_rptr_clear();
				printk("not repeater.\n");
			}

printk("((((((((((((((((((((((((((((((((((((((((A1:1))))))))))))))))))))))))))))))))))))))))\n");
			// Exange AN
			for(temp_hdcp[8]=0; temp_hdcp[8]<8; ++temp_hdcp[8]) {		 	   
			//	temp_hdcp[temp_hdcp[8]] = rand()%256;
				temp_hdcp[temp_hdcp[8]] = jiffies%256;
			}
		    hdmi_tx_write_an(temp_hdcp);
		    downstream_rx_write_an(temp_hdcp);

printk("((((((((((((((((((((((((((((((((((((((((A1:2))))))))))))))))))))))))))))))))))))))))\n");
			// Exange AKSV
		    if(!hdmi_tx_read_aksv(temp_hdcp)) {
			
				if(!fake_hdcp) {
					hdcp_state = A0_WAIT_FOR_ACTIVE_RX;
					hdcp_status |= HDCP_ERROR_AKSV;
					break;
				}
				else {
					memset(temp_hdcp, 0x5A, 5);
				}
			}
		    downstream_rx_write_aksv(temp_hdcp);
printk("((((((((((((((((((((((((((((((((((((((((A1:3))))))))))))))))))))))))))))))))))))))))\n");

			// Exange BKSV
		    if(!downstream_rx_read_bksv(temp_hdcp)) {
				hdcp_state = A0_WAIT_FOR_ACTIVE_RX;
				hdcp_status |= HDCP_ERROR_BKSV;
				break;
			}
		   	hdmi_tx_write_bksv(temp_hdcp);
			if(pbksv_bcaps3) memcpy(&pbksv_bcaps3[0], temp_hdcp, 5);
		
printk("$$$$$$$$$$$$$$$$$$$$$$$$$state = A2.\n");
		    hdcp_state = A2_COMPUTATIONS;
			break;

		// A2 state -- Computations
		//          -- Wait 150ms for R0 update (min 100ms)
		case A2_COMPUTATIONS:

printk("((((((((((((((((((((((((((((((((((((((((A2:))))))))))))))))))))))))))))))))))))))))\n");
		 	if(hdcp_timecount > 150/HDCP_TIMER_PERIOD) {
				if(hdmi_tx_ri_rdy()) {
					hdcp_timecount = 0;
					hdcp_state = A3_VALIDATE_RECEIVER;
				}
			}
			break;

		// A3 state -- Validate Receiver
		//          -- read and compare R0 from TX and RX
  		//          -- allow IIC traffic or R0 compare error in 200ms
		case A3_VALIDATE_RECEIVER: 
printk("((((((((((((((((((((((((((((((((((((((((A3:))))))))))))))))))))))))))))))))))))))))\n");
			if(!hdcp_validate_ri()) {
	 	    	if(hdcp_timecount > 200/HDCP_TIMER_PERIOD) {
					hdcp_timecount = 0;

					DBG_PRINTK(("ERROR: R0 check failed\r\n"));

					hdcp_state = A0_WAIT_FOR_ACTIVE_RX;	
					hdcp_status |= HDCP_ERROR_R0;
				}
			}
			else {
				hdcp_timecount = 0;
				hdcp_state = A6_TEST_FOR_REPEATER;
			}
			break;

		// A4 state -- Authenticated
		//          -- Disable mute
		case A4_AUTHENTICATED:
					
printk("((((((((((((((((((((((((((((((((((((((((A4:))))))))))))))))))))))))))))))))))))))))\n");
			// Start the HDCP Engine
			if(!fake_hdcp) hdmi_tx_hdcp_enable(); 
	
			// Disable mute for transmiter video 
			hdmi_tx_mute_disable();

			DBG_PRINTK(("Authenticated\r\n"));			

			hdcp_state = A5_LINK_INTEGRITY_CHECK;
			break;

		// A5 state -- Link Integrity Check every second
		//          -- HDCP Engine must be started
  		//          -- read and compare RI from RX and TX
		case A5_LINK_INTEGRITY_CHECK:

printk("((((((((((((((((((((((((((((((((((((((((A5:))))))))))))))))))))))))))))))))))))))))\n");
			if(ri_check) {
				ri_check = 0;

				if(!hdcp_validate_ri()) {
					if(!hdcp_validate_ri()) {

						// enable mute for transmiter video and audio
						hdmi_tx_mute_enable();

						// Disable the HDCP Engine
						hdmi_tx_hdcp_disable(); 

						DBG_PRINTK(("ERROR: Ri check failed\r\n"));

						hdcp_state = A0_WAIT_FOR_ACTIVE_RX;
						hdcp_status |= HDCP_ERROR_RI;
					}
				}
			}
/*
 	    	if(hdcp_timecount > 2000/HDCP_TIMER_PERIOD) {	// Wait for 2 second
				hdcp_timecount = 0;

				if(!hdcp_validate_ri()) {

					if(ri_check) { // RI_Failed_Two

						// enable mute for transmiter video and audio
						hdmi_tx_mute_enable();

						// Disable the HDCP Engine
						hdmi_tx_hdcp_disable(); 

						DBG_PRINTK(("ERROR: Ri check failed\r\n"));

						hdcp_state = A0_WAIT_FOR_ACTIVE_RX;
						hdcp_status |= HDCP_ERROR_Ri;
					}
					else {
						DBG_PRINTK(("WARNING: Ri check failed\r\n"));

						ri_check = 1;
						hdcp_timecount = 1500/HDCP_TIMER_PERIOD;
					}
				}
				else {
					ri_check = 0;
				}
			}
*/
			break;

		// A6 state -- Test For Repeater
		//          -- REPEATER     : Enter the WAIT_RX_RDY state;
		//          -- NON-REPEATER : Enter the AUTHENTICATED state
		case A6_TEST_FOR_REPEATER:   

printk("((((((((((((((((((((((((((((((((((((((((A6:))))))))))))))))))))))))))))))))))))))))\n");
			temp_hdcp[0] = downstream_rx_bcaps();
			if(pbksv_bcaps3) pbksv_bcaps3[5] = temp_hdcp[0];

			if (temp_hdcp[0] & 0x40) {	// REPEATER	
				hdcp_state = A8_WAIT_FOR_READY;
			}
			else {						// NON-REPEATER
				hdcp_state = A4_AUTHENTICATED;
			}
			break;

		// A8 state -- Wait for READY
		//          -- read BCAPS and check READY bit continuously
		//          -- time out while 5-second period exceeds
		case A8_WAIT_FOR_READY:   

printk("((((((((((((((((((((((((((((((((((((((((A8: wait for ready))))))))))))))))))))))))))))))))))))))))\n");
			temp_hdcp[0] = downstream_rx_bcaps();
			if(pbksv_bcaps3) pbksv_bcaps3[5] = temp_hdcp[0];

			if (temp_hdcp[0] & 0x20) {
				hdcp_timecount = 0;
				hdcp_state = A9_READ_KSV_LIST;
			}
			else {
				if(hdcp_timecount > 5000/HDCP_TIMER_PERIOD) {
					hdcp_timecount = 0;

					DBG_PRINTK(("ERROR: Repeater check READY bit time-out\r\n"));

					hdcp_state = A0_WAIT_FOR_ACTIVE_RX;	
					hdcp_status |= HDCP_ERROR_REPEATERRDY;
				}
			}
			break;

		// A9 state -- Read KSV List
		//          -- compute and validate SHA-1 values
		case A9_READ_KSV_LIST:
			
printk("((((((((((((((((((((((((((((((((((((((((A9: read ksv list))))))))))))))))))))))))))))))))))))))))\n");
			downstream_rx_read_bstatus(temp_hdcp);
			if(pbksv_bcaps3) memcpy(&pbksv_bcaps3[6], temp_hdcp, 2);

			if(!(temp_hdcp[0] & 0x80) && !(temp_hdcp[1] & 0x08)) {
				if(hdcp_compute_sha_message_digest(temp_hdcp[0], temp_hdcp[1])) {
					hdcp_state = A4_AUTHENTICATED;
					break;
				}
				else {
					hdcp_status |= HDCP_ERROR_REPEATERSHA;
				}
			}
			else {
				hdcp_status |= HDCP_ERROR_REPEATERMAX;
			}

			DBG_PRINTK(("ERROR: Repeater HDCP SHA check failed\r\n"));

			hdcp_state = A0_WAIT_FOR_ACTIVE_RX;	
			break;
	}

	return hdcp_state;
}

//----------------------------------------------------------------------------------------------------------------------

unsigned char hdcp_validate_ri(void)
{
	unsigned short temp_ri_tx, temp_ri_rx;
	if(!hdmi_tx_read_ri((unsigned char *)&temp_ri_tx)) return 0;		// Read form Tx is fast, do it first
	if(!downstream_rx_read_ri((unsigned char *)&temp_ri_rx)) return 0;	// Read form Rx is slow, do it second
//	if(temp_ri_tx != temp_ri_rx) DBG_PRINTK(("RI_Tx=0x%0.4X, RI_Rx=0x%0.4X\r\n", (int)Temp_RI_Tx, (int)temp_ri_rx));
	if(fake_hdcp) return 1;
	return (temp_ri_tx == temp_ri_rx);
}

//--------------------------------------------------------------------------------------------------

//
// NOTE : The following SHA calculation subroutine has not been verified in the
//        real environment. It has been evaluated by some debugging procedure.
//        The auther is not responsible to ensure the functionality.
//
unsigned char hdcp_compute_sha_message_digest(unsigned char hdcp_dev_count, unsigned char hdcp_depth) 
{
	int i;
	unsigned long *sha_h;
  
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Calculate SHA Value
	//

	sha_initial();

	//
	// Step 1
	// Push all KSV FIFO to SHA caculation
	//

	// Read KSV (5 byte) one by one and check the revocation list
	for(i=0; i<hdcp_dev_count; ++i) {

		// Get KSV from FIFO
		if(!downstream_rx_read_ksv_fifo(temp_hdcp, i, hdcp_dev_count)) {
			return 0;
		}

		// Save FIFO 
		if(pksvlist && ksvlistnumber) {
			if(i < ksvlistnumber) memcpy(pksvlist+(i*5), temp_hdcp, 5);
		}

		// Push KSV to the SHA block buffer (Total 5 bytes)
		sha_push_data(temp_hdcp, 5);
	}
	if(hdcp_dev_count == 0) {
		downstream_rx_read_ksv_fifo(temp_hdcp, 0, 1);
	}

	//
	// Step 2
	// Push BSTATUS, M0, and EOF to SHA caculation
	//

	// Get the BSTATUS, M0, and EOF
	temp_hdcp[0] = hdcp_dev_count;	// temp_hdcp[0]   = BStatus, LSB
	temp_hdcp[1] = hdcp_depth;		// temp_hdcp[1]   = BStatus, MSB
	hdmi_tx_read_m0(temp_hdcp+2);	// temp_hdcp[2:9] = Read M0 from TX
	if(psha_m0) memcpy(psha_m0+20, (unsigned char*)temp_hdcp+2, 8);

	// Push the BSTATUS, and M0 to the SHA block buffer (Total 10 bytes)
	sha_push_data(temp_hdcp, 10);

	//
	// Step 3
	// Push the final block with length to SHA caculation
	//

	sha_h = sha_get_sha_digest();

	//
	// SHA complete
	//////////////////////////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////////////////////////
	// Compare the SHA value
	//

	// read RX SHA value
	downstream_rx_read_sha1_hash((unsigned char*)temp_sha_h); 
	if(psha_m0) memcpy(psha_m0, (unsigned char*)temp_sha_h, 20);
	DBG_PRINTK(("Rx_sha_h: "));  
#ifdef DBG
	for(i=0; i<20; i+=4) {		
		DBG_PRINTK(("0x%0.2X%0.2X%0.2X%0.2X ", (int)(((PBYTE)temp_sha_h)[i+3]), (int)(((PBYTE)temp_sha_h)[i+2]), (int)(((PBYTE)temp_sha_h)[i+1]), (int)(((PBYTE)temp_sha_h)[i+0])));
	}
	DBG_PRINTK(("\r\n"));
#endif
	// compare the TX/RX SHA value
	if( (hdcp_dev_count & 0x80) || (hdcp_depth & 0x08) ) {
		DBG_PRINTK(("Max Cascade or Max Devs exceeded\r\n"));
		return 0;
	}
	else if( (sha_h[0] != temp_sha_h[0]) || (sha_h[1] != temp_sha_h[1]) || (sha_h[2] != temp_sha_h[2]) || (sha_h[3] != temp_sha_h[3]) || (sha_h[4] != temp_sha_h[4]) ) {
		DBG_PRINTK(("SHA Digit Unmatch\r\n"));
  		return 0;
	}
	else {
		DBG_PRINTK(("SHA Digit Match\r\n"));
		return 1;
	}

	//
	// Return the compared result
	//////////////////////////////////////////////////////////////////////////////////////////////

}

//--------------------------------------------------------------------------------------------------
// SHA Implementation
//--------------------------------------------------------------------------------------------------

unsigned long sha_h[5];
unsigned char sha_block[64];	 	// 16*4
unsigned char sha_block_index;
unsigned char copysize;
unsigned int sha_length;

void sha_initial(void)
{
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Calculate SHA Value
	//

	// initial the SHA variables
	sha_h[0] = 0x67452301;
	sha_h[1] = 0xEFCDAB89;
	sha_h[2] = 0x98BADCFE;
	sha_h[3] = 0x10325476;
	sha_h[4] = 0xC3D2E1F0;

	// Clean the SHA Block buffer
	memset(sha_block, 0, 64);
	sha_block_index = 0;

	sha_length = 0;
}

void sha_push_data(unsigned char *pdata, unsigned char size)
{
	int i;
	sha_length += size;

	while(size) {
		// Push Data to the SHA block buffer
		copysize = min((64-sha_block_index), size);
		memcpy(sha_block+sha_block_index, pdata, copysize);
		sha_block_index += copysize;
		pdata += copysize;
		size -= copysize;

		if(sha_block_index >= 64) { // The SHA block buffer Full

        // add by Eric_Lu

			// Swap the sequence of SHA Block (The little-endian to big-endian)
			unsigned char swap_temp;
			for(i=0; i<64; i+=4) {
			   
				swap_temp = sha_block[i+0];
				sha_block[i+0] = sha_block[i+3];
				sha_block[i+3] = swap_temp;

				swap_temp = sha_block[i+1];
				sha_block[i+1] = sha_block[i+2];
				sha_block[i+2] = swap_temp;
			}
         // add end
			// Do SHA caculation for this SHA block buffer
			sha_calculation(sha_h, (unsigned long*)sha_block);
			memset(sha_block, 0, 64);				
	
			sha_block_index = 0; // Reset the Index
		}
	}
}

unsigned long *sha_get_sha_digest(void)
{
	int i;
	unsigned char swap_temp;
	sha_block[sha_block_index++] = 0x80;	// Set EOF

	if((64 - sha_block_index) < 2) {
		memset(sha_block, 0, 64);
	}
	sha_length *= 8; 
	sha_block[62] = (sha_length >> 8) & 0xFF; 			// Pad with Length MSB
	sha_block[63] = sha_length & 0xFF;  				// Pad with Length LSB

        // add by Eric_Lu
	// Swap the sequence of SHA Block (The little-endian to big-endian)
	for(i=0; i<64; i+=4) {
	   
		swap_temp = sha_block[i+0];
		sha_block[i+0] = sha_block[i+3];
		sha_block[i+3] = swap_temp;

		swap_temp = sha_block[i+1];
		sha_block[i+1] = sha_block[i+2];
		sha_block[i+2] = swap_temp;
	}
        // add end

	// Do SHA caculation for final SHA block
	sha_calculation(sha_h, (unsigned long*)sha_block);
	
	// Swap the sequence of sha_h (The big-endian to little-endian)
	DBG_PRINTK(("sha_h:    ")); 
	for(i=0; i<20; i+=4) {
	   
		temp_hdcp[0] = ((unsigned char*)sha_h)[i+0];
		((unsigned char*)sha_h)[i+0] = ((unsigned char*)sha_h)[i+3];
		((unsigned char*)sha_h)[i+3] = temp_hdcp[0];

		temp_hdcp[0] = ((unsigned char*)sha_h)[i+1];
		((unsigned char*)sha_h)[i+1] = ((unsigned char*)sha_h)[i+2];
		((unsigned char*)sha_h)[i+2] = temp_hdcp[0];

		DBG_PRINTK(("0x%02X%02X%02X%02X ", (int)(((unsigned char*)sha_h)[i+3]), (int)(((unsigned char*)sha_h)[i+2]), (int)(((unsigned char*)sha_h)[i+1]), (int)(((unsigned char*)sha_h)[i+0])));
	}
	DBG_PRINTK(("\r\n")); 

	return sha_h;
}

void sha_calculation(unsigned long psha_h[5], unsigned long psha_w1[16])
{
	unsigned char i;
	unsigned long temp;

	// =========================================================
	//
	// STEP (c) : Let A = H0, B = H1, C = H2, D = H3, E = H4
	//
	temp_sha_h[0] = psha_h[0];
	temp_sha_h[1] = psha_h[1];
	temp_sha_h[2] = psha_h[2];
	temp_sha_h[3] = psha_h[3];
	temp_sha_h[4] = psha_h[4];
	//
	// =========================================================    
													
	// =========================================================
	//
	// STEP (d) : FOR t = 0 to 79 DO
	//              1. temp = S5(A) + Ft(B,C,D) + E + Wt + Kt
	//              2. E = D; D = C; C = S30(B); B = A; A = temp;
	//
	for (i = 0; i <= 79; i++) {
		// Update the Message Word while loop time >= 16
		if (i >= 16) {
			// tword = psha_w1[tm03] ^ psha_w1[tm08] ^ psha_w1[tm14] ^ psha_w1[tm16];                   
			temp = psha_w1[(i + 13) % 16] ^ psha_w1[(i + 8) % 16] ^ psha_w1[(i + 2) % 16] ^ psha_w1[i % 16];
			psha_w1[i % 16] = (temp << 1) | (temp >> 31);
		}

		// Calculate first equation
		temp = psha_w1[i % 16];

	    temp += ((temp_sha_h[0] << 5) | (temp_sha_h[0] >> 27));
	
	    if (i <= 19)      temp += ((temp_sha_h[1] & temp_sha_h[2]) | (~temp_sha_h[1] & temp_sha_h[3])) + 0x5A827999;
	    else if (i <= 39) temp += (temp_sha_h[1] ^ temp_sha_h[2] ^ temp_sha_h[3]) + 0x6ED9EBA1;
	    else if (i <= 59) temp += ((temp_sha_h[1] & temp_sha_h[2]) | (temp_sha_h[1] & temp_sha_h[3]) | (temp_sha_h[2] & temp_sha_h[3])) + 0x8F1BBCDC;
	    else              temp += (temp_sha_h[1] ^ temp_sha_h[2] ^ temp_sha_h[3]) + 0xCA62C1D6;

	    temp += temp_sha_h[4];

    	// Update the Value A/B/C/D/E
    	temp_sha_h[4] = temp_sha_h[3];
    	temp_sha_h[3] = temp_sha_h[2];
    	temp_sha_h[2] = ((temp_sha_h[1] << 30) | (temp_sha_h[1] >> 2));
    	temp_sha_h[1] = temp_sha_h[0];
    	temp_sha_h[0] = temp;
	}
	//
	// =========================================================

	// =========================================================
	//
	// STEP (e) : H0 = H0 + A; H1 = H1 + B; H2 = H2 + C; H3 = H3 + D; H4 = H4 + E;
	//
	psha_h[0] += temp_sha_h[0];
	psha_h[1] += temp_sha_h[1];
	psha_h[2] += temp_sha_h[2];
	psha_h[3] += temp_sha_h[3];
	psha_h[4] += temp_sha_h[4];
	//
	// =========================================================
}

