
#include "tcp_server.h"

#define SERVER_LISTEN_PORT 7

static err_t tcp_server_accepted(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_server_error(void *arg, err_t err);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, uint16_t len);
static void tcp_server_write(struct tcp_pcb *tpcb, struct tcp_server_state_struct *tss);
static void tcp_server_close_connection(struct tcp_pcb *tpcb, struct tcp_server_state_struct *tss);

static struct tcp_pcb *pcb_server;

err_t init_tcp_server(void) 
{
    err_t err;

    pcb_server = tcp_new();

    if (pcb_server == NULL)
    {
        memp_free(MEMP_TCP_PCB, pcb_server);
        return ERR_MEM;
    }

    err = tcp_bind(pcb_server, IP_ADDR_ANY, SERVER_LISTEN_PORT);
    if (err != ERR_OK)
    {
        memp_free(MEMP_TCP_PCB, pcb_server);
        return err;
    }

    pcb_server = tcp_listen(pcb_server);	//listen
    tcp_accept(pcb_server, tcp_server_accepted);	//register accept callback

    return ERR_OK;
}

static err_t tcp_server_accepted(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    struct tcp_server_state_struct* tss;

    tcp_setprio(newpcb, TCP_PRIO_NORMAL);

    tss = (struct tcp_server_state_struct*)mem_malloc(sizeof(struct tcp_server_state_struct));

    if (tss == NULL) //lack of memory
    {
        tcp_server_close_connection(newpcb, tss);
        return ERR_MEM;
    }

    tss->state = ES_ACCEPTED;
    tss->pcb = newpcb;
    tss->retries = 0;
    tss->p = NULL;

    tcp_arg(  newpcb, tss               );
    tcp_recv( newpcb, tcp_server_recv   );
    tcp_err(  newpcb, tcp_server_error  );
    tcp_poll( newpcb, tcp_server_poll, 0);
    tcp_sent( newpcb, tcp_server_sent   );

    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{

	struct tcp_server_state_struct* tss;
    err_t ret_err;

    tss = (struct tcp_server_state_struct*) arg;

    if (p == NULL)
    {
        tss->state = ES_CLOSING;
        if (tss->p == NULL)
        {
            tcp_server_close_connection(tpcb, tss);
        }
        else
        {
            tcp_sent(tpcb, tcp_server_sent);
            tcp_server_write(tpcb, tss);
        }
        ret_err = ERR_OK;
    }
    else if (err != ERR_OK)
    {
        if (p != NULL)
        {
            tss->p = NULL;
            pbuf_free(p);
        }
            ret_err = err;
    }
    else if (tss->state == ES_ACCEPTED)
    {
        tss->state = ES_RECEIVED;
        tss->p = p;
        tcp_sent(tpcb, tcp_server_sent);
        tcp_server_write(tpcb, tss);
        ret_err = ERR_OK;
    }
    else if (tss->state == ES_RECEIVED)
    {
        if (tss->p == NULL)
        {
            tss->p = p;
            tcp_server_write(tpcb, tss);
        }
        else
        {
            struct pbuf *ptr = tss->p;
            pbuf_chain(ptr, p);
        }
        ret_err = ERR_OK;
    }
    else if (tss->state == ES_CLOSING)
    {
        tcp_recved(tpcb, p->tot_len);
        tss->p = NULL;
        pbuf_free(p);
        ret_err = ERR_OK;
    }
    else
    {
        tcp_recved(tpcb, p->tot_len);
        tss->p = NULL;
        pbuf_free(p);
        ret_err = ERR_OK;
    }
    return ret_err;
}

static void tcp_server_error(void *arg, err_t err)
{
    struct tcp_server_state_struct *tss;

    tss = (struct tcp_server_state_struct*) arg;
    if (tss != NULL)
    {
        mem_free(tss);	//free es structure
    }

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); //turn on blue LED when there's error.
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
    struct tcp_server_state_struct *tss;
    tss = (struct tcp_server_state_struct*) arg;

    if (tss == NULL) //if there's no es structure
    {
        tcp_abort(tpcb); //abort connection
        return ERR_ABRT;
    }

    if (tss->p != NULL) //if there's data to send
    {
        tcp_sent(tpcb, tcp_server_sent); //register send callback
        tcp_server_write(tpcb, tss); //send data
    }
    else //no data to send
    {
        if (tss->state == ES_CLOSING)
        {
            tcp_server_close_connection(tpcb, tss);		//close connection
        }
    }

    return ERR_OK;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, uint16_t len)
{
    struct tcp_server_state_struct *tss;

    tss = (struct tcp_server_state_struct*) arg;
    tss->retries = 0;

    if (tss->p != NULL) //if there's data to send
    {
        tcp_sent(tpcb, tcp_server_sent); //register send callback
        tcp_server_write(tpcb, tss); //send data
    }
    else //no data to send
    {
        if (tss->state == ES_CLOSING)
        {
            tcp_server_close_connection(tpcb, tss); //close connection
        }
    }
    return ERR_OK;
}

typedef struct {
	char n1;
	char n2;
	char n3;
	char n4;
} data_to_send;

typedef struct {
	char n1;
	char n2;
	char n3;
	char n4;
} input_data_struct;

typedef struct {
    int x_en;
    int y_en;
    float x_position;
    float y_position;
    float x_velocity;
    float y_velocity;
} config_packet_struct;

config_packet_struct config_packet = {0};
extern data_packet_struct Data_Packet[2];
extern int  Previous;

static void tcp_server_write(struct tcp_pcb *tpcb, struct tcp_server_state_struct *tss)
{
    struct pbuf *ptr;
    err_t wr_err = ERR_OK;
    err_t or_err = ERR_OK;


    // while no error, data to send, data size is smaller than the size of the send buffer
    while ( ( wr_err     == ERR_OK )    &&
            ( tss->p      != NULL   )    &&
            ( tss->p->len <= tcp_sndbuf(tpcb) ) )
    {
        ptr = tss->p;

        //static input_data_struct input;
        memcpy(&config_packet, ptr->payload, sizeof(config_packet_struct));
        if(config_packet.x_en == 1){
            //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        // TODO substitue ptr_payload with your data
        //      and also make it wait for this data.
#if 0
        static data_packet_struct real;
        for (int iter = 0; iter < 100; iter++)
        {
            real.carts_pos_x[iter]          = (float)(iter%100);
            real.carts_pos_y[iter]          = (float)(iter%100);
            real.carts_vel_x[iter]          = (float)(iter%100);
            real.carts_vel_y[iter]          = (float)(iter%100);
            real.carts_acc_x[iter]          = (float)(iter%100);
            real.carts_acc_y[iter]          = (float)(iter%100);
            real.mpu9250_acce_x[iter]       = (float)(iter%100);
            real.mpu9250_acce_y[iter]       = (float)(iter%100);
            real.mpu9250_acce_z[iter]       = (float)(iter%100);
            real.mpu9250_gyro_x[iter]       = (float)(iter%100);
            real.mpu9250_gyro_y[iter]       = (float)(iter%100);
            real.mpu9250_gyro_z[iter]       = (float)(iter%100);
            real.mpu6886_acce_x[iter]       = (float)(iter%100);
            real.mpu6886_acce_y[iter]       = (float)(iter%100);
            real.mpu6886_acce_z[iter]       = (float)(iter%100);
            real.mpu6886_gyro_x[iter]       = (float)(iter%100);
            real.mpu6886_gyro_y[iter]       = (float)(iter%100);
            real.mpu6886_gyro_z[iter]       = (float)(iter%100);
            real.pitch_no_filter[iter]      = (float)(iter%100);
            real.roll_no_filter[iter]       = (float)(iter%100);
            real.pitch_complementary[iter]  = (float)(iter%100);
            real.roll_complementary[iter]   = (float)(iter%100);
            real.pitch_alfa_beta[iter]      = (float)(iter%100);
            real.roll_alfa_beta[iter]       = (float)(iter%100);
            real.pitch_kalman[iter]         = (float)(iter%100);
            real.roll_kalman[iter]          = (float)(iter%100);
            real.pitch[iter]                = (float)(iter%100);
            real.roll[iter]                 = (float)(iter%100);
        }
        real.carts_pos_x[0] = 11.0;
        real.roll[99] = 13.0;
        real.sytem_status.carts.steps = 33;
        HAL_Delay(900);
#endif
        while( Previous == PACKET_NONE );
        HAL_GPIO_TogglePin(STATUS_ETH_GPIO_Port, STATUS_ETH_Pin);
        wr_err = tcp_write(tpcb, &Data_Packet[Previous], sizeof(data_packet_struct), TCP_WRITE_FLAG_COPY); //send data
        or_err = tcp_output(tpcb);
        HAL_GPIO_TogglePin(STATUS_ETH_GPIO_Port, STATUS_ETH_Pin);
        Previous = PACKET_NONE;

        if (or_err != ERR_OK) {
        	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }

        if (wr_err == ERR_OK)
        {
            uint16_t plen  ;
            uint8_t  freed ;

            plen = ptr->len;
            tss->p = ptr->next;

            if (tss->p != NULL) //there's chained buffer to send
            {
                pbuf_ref(tss->p);	//increase reference counter
            }

            do
            {
                freed = pbuf_free(ptr);		//free old buffer
            }
            while (freed == 0);

            tcp_recved(tpcb, plen);			//advertise window size
        }
        else
        {
            tss->p = ptr;  //fail to send, recover buffer pointer
            tss->retries++;  //increase counter
        }
    }
}

static void tcp_server_close_connection(struct tcp_pcb *tpcb, struct tcp_server_state_struct *tss)
{
    /* clear callback functions */
    tcp_arg(  tpcb, NULL    ) ;
    tcp_sent( tpcb, NULL    ) ;
    tcp_recv( tpcb, NULL    ) ;
    tcp_err(  tpcb, NULL    ) ;
    tcp_poll( tpcb, NULL, 0 ) ;

    if (tss != NULL)
    {
        mem_free(tss);		//free es structure
    }

    tcp_close(tpcb);		//close connection
}
