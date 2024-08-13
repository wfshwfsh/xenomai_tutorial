#include <stdio.h>

#define BUF_LEN 8
#define NEXT_PLACE(i) ((i+1)&(BUF_LEN-1))

static unsigned char txbuf[BUF_LEN];
static int tx_buf_r = 0;
static int tx_buf_w = 0;


static int is_txbuf_empty(void)
{
	return (tx_buf_r == tx_buf_w);
}

static int is_txbuf_full(void)
{
	return (NEXT_PLACE(tx_buf_w) == tx_buf_r);
}

static int txbuf_put(unsigned char val)
{
	if(is_txbuf_full())
		return -1;

	txbuf[tx_buf_w] = val;
	tx_buf_w = NEXT_PLACE(tx_buf_w);
	return 0;
}

static int txbuf_get(unsigned char *pval)
{
	if(is_txbuf_empty())
		return -1;
	
	*pval = txbuf[tx_buf_r];
	tx_buf_r = NEXT_PLACE(tx_buf_r);
	return 0;
}

static int txbuf_count(void)
{
	if(tx_buf_w >= tx_buf_r)
		return (tx_buf_w - tx_buf_r);
	else
		return BUF_LEN-tx_buf_r+tx_buf_w;
}


static void txbuf_print()
{
	int i;
	int cnt_buf = txbuf_count();

	printf("#########\n");
	printf("buf: ");
	for(i=tx_buf_r;cnt_buf>0;i++,cnt_buf--){
		
		printf("%d ", txbuf[i&(BUF_LEN-1)]);
	}
	printf("\n");
}

int main(int argc, char **argv)
{
	unsigned char data;

	txbuf_put(3);
	txbuf_put(4);
	txbuf_put(6);
	txbuf_put(8);
	
        txbuf_print();
	if(0 != txbuf_get(&data)){
		printf("failed");
	}else{
		printf("kick data %d\n", data);
	}
	
	txbuf_print();


	txbuf_put(5);
	txbuf_put(5);
	
	txbuf_put(7);
	txbuf_put(7);

	printf("99999999999999\n");
	txbuf_print();
	if(txbuf_put(9) != 0){
		printf("full\n");
		printf("cnt=%d, w=%d, r=%d\n",  txbuf_count(), tx_buf_w, tx_buf_r);
	}
	
	if(0 != txbuf_get(&data)){
                printf("failed");
        }else{
                printf("kick data %d\n", data);
        }	
	txbuf_print();
	if(txbuf_put(15) != 0){
		printf("full\n");
	}

	txbuf_print();

	return 0;
}
