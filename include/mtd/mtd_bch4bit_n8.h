

#define nn_max  32768        	/* Length of codeword, n = 2**m - 1 */
#define tt_max  32          	/* Number of errors that can be corrected */
#define kk_max  32768        	/* Length of information bit, kk = nn - rr  */
#define rr_max  32		/* Number of parity checks, rr = deg[g(x)] */
#define BCH4_DEBUG 0

//Encoder
extern int mm, nn, kk, tt, rr;	// BCH code parameters
extern int T_G_R[32][32];		// Parallel lookahead table
extern int T_G_R_PACK[32];

// Decoder
//extern int ttx2;		// 2t
extern int alpha_to[], index_of[] ;	// Galois field


//Enc, Dec functions
extern void do_bch_encode (unsigned char *inbuf, unsigned char *paritybuf, int len);
extern void do_bch_correct_data (unsigned char *inbuf, unsigned int *errpos);
extern int  do_bch_decode (unsigned char *inbuf, unsigned char *paritybuf, unsigned int *error_pos, int len);
extern void do_bch_decode_and_correct (unsigned char *inbuf, unsigned char *paritybuf, int len);
