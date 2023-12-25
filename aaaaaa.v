
//	input	[7:0]	rx_data,
//	input 			btn_1,
//	input			rx_int,
//	output			uart_tx,
//	input			clk_bps,
//	output			bps_start
module mid_uart_tx(
        input clk,rst,
        input btn,
        output            out_tx
//       , output uart_tx,back
    );
    parameter LENGH = 11;
    parameter DATA = 88'h69206C696B652046504741;
    
reg btn0, btn1, btn2,btn_en;
    wire btn_rise,btn_mid;
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            btn0 <= 0;
            btn1 <= 0;
            btn2 <= 0;
        end else begin
            btn0 <= btn;
            btn1 <= btn0;
            btn2 <= btn1;
        end
        
    end
    
    assign btn_mid = (btn1 & !btn2)|(btn0 & !btn2); // Detect rising edge
//    assign  btn_mid= btn0 & btn_rise; // btn_mid goes high on rising edge of btn

    

    
    
    wire back;
    reg en;
    reg [8:0] cnt;
    reg [88:0] send_data ;


    always @(posedge clk or negedge rst)begin
        if(!rst)begin
            
           send_data = DATA;
        end else
        if(btn_mid)begin
            en <= 1;
        end else
        if(cnt == LENGH)begin
            en <= 0 ;
        end
    
    end
    
    reg en_tx,back_reg;
    
    reg [7:0]sending_data;
    integer sending_cnt; 
    always @(posedge clk or negedge rst)begin
        if(!rst)begin
            en_tx <=0 ;
            cnt <= 0 ;
            sending_cnt <= LENGH * 8 -8 ;
        end else if(en)begin
            if(back)back_reg = back;
        
            if(btn_mid)btn_en <= btn_mid;
            if(back_reg || btn_en)begin
                en_tx <= 1;
                btn_en <= 0;
                back_reg <= 0;
                cnt = cnt +1 ;
                sending_cnt <= sending_cnt - 8 ;
                sending_data[0] <= send_data[sending_cnt];
                sending_data[1] <= send_data[sending_cnt+1];
                sending_data[2] <= send_data[sending_cnt+2];
                sending_data[3] <= send_data[sending_cnt+3];
                sending_data[4] <= send_data[sending_cnt+4];
                sending_data[5] <= send_data[sending_cnt+5];
                sending_data[6] <= send_data[sending_cnt+6];
                sending_data[7] <= send_data[sending_cnt+7];
          
            end else begin
                        en_tx <= 0;
                    end
            
        end
    
    end
    wire [7:0]uart_tx;
    assign uart_tx = sending_data;
//    module speed_setting
//    #(
//        parameter BPS_SET      =   96,  //波特率
//        parameter CLK_PERIORD  =   40   //时钟周期40ns(25MHz)
//    )
//    (
//        input    clk,        //25MHz主时钟
//        input    rst_n,        //低电平复位信号
//        input    bps_start,    //接收到数据后，波特率时钟启动信号置位
//        output    clk_bps        //clk_bps的高电平为接收或者发送数据位的中间采样点
//    );
    wire bps_start,clk_bps;
    speed_setting spd(
    .clk(clk),
    .rst_n(rst),
    .bps_start(bps_start),
    .clk_bps(clk_bps)
    );
    	//UART发送数据处理
//module my_uart_tx
//        (
//            input            clk,
//            input            rst,
//            input    [7:0]    tx_data,
//            input             btn_1,
//            output            uart_tx,
//            input            clk_bps,
//            output            bps_start,
//            output             back
//        );

my_uart_tx    my_uart_tx
(        
    .clk            (clk        ),    //发送数据模块
    .rst          (rst      ),
    .back           (back),
    .tx_data        (uart_tx),
    .uart_tx        (out_tx        ),
    .clk_bps        (clk_bps       ),
    .bps_start      (bps_start     ),
    .btn_1            (en_tx)
);
endmodule
