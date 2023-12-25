
module my_uart_rx1
(
	input			clk,		//25MHz主时钟
	input			rst,		//低电平复位信号
	input			uart_rx,	//接收数据信号
	output	[7:0]	rx_data,	//接收数据寄存器，保存直至下一个数据来到
    
	input			clk_bps,	//clk_bps的高电平为接收或者发送数据位的中间采样点
	output			bps_start,	//接收到数据后，波特率时钟启动信号置位
	output 	reg	[14:0] led,
	 output reg [3:0] next_state,
	    output reg match
);






//----------------------------------------------------------------
reg		uart_rx0,uart_rx1,uart_rx2,uart_rx3;	//接收数据寄存器
wire	neg_uart_rx;							
 
always @ (posedge clk or negedge rst)
begin 
	if(!rst) 
    begin
		uart_rx0 <= 1'b0;
		uart_rx1 <= 1'b0;
		uart_rx2 <= 1'b0;
		uart_rx3 <= 1'b0;
	end
	else 
    begin
		uart_rx0 <= uart_rx;
		uart_rx1 <= uart_rx0;
		uart_rx2 <= uart_rx1;
		uart_rx3 <= uart_rx2;
	end
end

assign neg_uart_rx = uart_rx3 & ~uart_rx2 ;	//接收到下降沿后neg_uart_rx置高一个时钟周期

//----------------------------------------------------------------
reg 		bps_start_r;
reg	[3:0] 	num;	
reg 		rx_int;		//接收数据中断信号,接收到数据期间始终为高电平

always @ (posedge clk or negedge rst)
begin
	if(!rst)  
    begin
		bps_start_r <= 1'bz;
		rx_int 		<= 1'b0;
	end
	else if(neg_uart_rx) 
    begin	                    //接收到串口接收线uart_rx的下降沿标志信号  
		bps_start_r <= 1'b1;	//启动串口准备数据接收   
		rx_int 		<= 1'b1;	//接收数据中断信号使能
	end
	else if(num == 4'd9) 
    begin	                    //接收完有用数据信息
		bps_start_r <= 1'b0;	//数据接收完毕，释放波特率启动信号
		rx_int 		<= 1'b0;	//接收数据中断信号关闭
	end
end

assign bps_start = bps_start_r;

//----------------------------------------------------------------
reg	[7:0]	rx_data_r;		//串口接收数据寄存器
reg	[7:0]	rx_temp_data;	//当前接收数据寄存器
reg charge;
always @ (posedge clk or negedge rst)
begin
	if(!rst) 
    begin
		rx_temp_data 	<= 8'd0;
		num 			<= 4'd0;
		rx_data_r 		<= 8'd0;
		led				<= 8'b0;
		charge        <= 0;
	end
	else if(rx_int) 
   begin	                //接收数据处理

		if(clk_bps) 
        begin	            	
			num <= num+1'b1;
            
			case (num)
				4'd1: begin
                    rx_temp_data[0] <= uart_rx;	//锁存第0bit
                    led[0] <= uart_rx; //将uart_rx赋给led的第0位
                end
				4'd2: begin
                    rx_temp_data[1] <= uart_rx;	//锁存第1bit
                    led[1] <= uart_rx; //将uart_rx赋给led的第1位
                end
				4'd3: begin
                    rx_temp_data[2] <= uart_rx;	//锁存第2bit
                    led[2] <= uart_rx; //将uart_rx赋给led的第2位
                end
				4'd4: begin
                    rx_temp_data[3] <= uart_rx;	//锁存第3bit
                    led[3] <= uart_rx; //将uart_rx赋给led的第3位
                end
				4'd5: begin
                    rx_temp_data[4] <= uart_rx;	//锁存第4bit
                    led[4] <= uart_rx; //将uart_rx赋给led的第4位
                end
				4'd6: begin
                    rx_temp_data[5] <= uart_rx;	//锁存第5bit
                    led[5] <= uart_rx; //将uart_rx赋给led的第5位
                end  
				4'd7: begin
                    rx_temp_data[6] <= uart_rx;	//锁存第6bit
                    led[6] <= uart_rx; //将uart_rx赋给led的第6位
                end 
				4'd8: begin
                    rx_temp_data[7] <= uart_rx;	//锁存第7bit
                    led[7] <= uart_rx; //将uart_rx赋给led的第7位
                end 
				default: ;
			endcase
		end
		else if(num == 4'd9) 
        begin		
            charge      <=1;
			num			<= 4'd0;			//接收到STOP位后结束,num清零
			rx_data_r	<= rx_temp_data;	//把数据锁存到数据寄存器rx_data中
			
//			led = rx_temp_data;
//			led <= rx_data_r;
		end else charge <= 0;
	end
end



assign rx_data = rx_data_r;	






//parameter IDLE = 4'b0000, I = 4'b0001, SPACE1 = 4'b0010, L = 4'b0011, 
//           I2 = 4'b0100, K = 4'b0101, E = 4'b0110, SPACE2 = 4'b0111, 
//           F = 4'b1000, P = 4'b1001, G = 4'b1010, A = 4'b1011, V = 4'b1100, R = 4'b1101, O = 4'b1110, L2 = 4'b1111;
//reg [3:0] state, next_state;

//initial next_state = IDLE;
//always @( posedge charge or negedge rst) begin
//    if (!rst) state <= IDLE;
//    else 
//    case (next_state)
//        IDLE: if (rx_data_r == 8'h69) next_state = I; else next_state = IDLE;
//        I: if (rx_data_r == 8'h20) next_state = SPACE1; else next_state = IDLE;
//        SPACE1: if (rx_data_r == 8'h6C) next_state = L; else next_state = IDLE;
//        L: if (rx_data_r == 8'h69) next_state = I2; else next_state = IDLE;
//        I2: if (rx_data_r == 8'h6B) next_state = K; else next_state = IDLE;
//        K: if (rx_data_r == 8'h65) next_state = E; else next_state = IDLE;
//        E: if (rx_data_r == 8'h20) next_state = SPACE2; else next_state = IDLE;
//        SPACE2: if (rx_data_r == 8'h46) next_state = F; else if (rx_data_r == 8'h76) next_state = V; else next_state = IDLE;
//        F: if (rx_data_r == 8'h50) next_state = P; else next_state = IDLE;
//        P: if (rx_data_r == 8'h47) next_state = G; else next_state = IDLE;
//        G: if (rx_data_r == 8'h41) next_state = A; else next_state = IDLE;
//        A: next_state = IDLE;
//        V: if (rx_data_r == 8'h65) next_state = E; else next_state = IDLE;
//        E: if (rx_data_r == 8'h72) next_state = R; else next_state = IDLE;
//        R: if (rx_data_r == 8'h69) next_state = I; else next_state = IDLE;
//        I: if (rx_data_r == 8'h6C) next_state = L2; else next_state = IDLE;
//        L2: if (rx_data_r == 8'h6F) next_state = O; else next_state = IDLE;
//        O: if (rx_data_r == 8'h67) next_state = G; else next_state = IDLE;
//        default: next_state = IDLE;
//    endcase
//end

//always @(posedge clk or negedge rst) begin
//    if (!rst)begin
//     match1 <= 0;
//     match2 <= 0;
//     end
//    else if ( next_state == A) match1 <= 1;
//     else if (next_state == G ) match2 <= 1;

//end



parameter IDLE = 4'b0000, I = 4'b0001, SPACE1 = 4'b0010, L = 4'b0011, 
           I2 = 4'b0100, K = 4'b0101, E = 4'b0110, SPACE2 = 4'b0111, 
           V = 4'b1000, E2 = 4'b1001, R = 4'b1010, I3 = 4'b1011, L2 = 4'b1100, O = 4'b1101, G = 4'b1110;
reg [3:0] state, next_state;

initial next_state = IDLE;
always @( posedge charge or negedge rst) begin
    if (!rst) state <= IDLE;
    else 
    case (next_state)
        IDLE: if (rx_data_r == 8'h69) next_state = I; else next_state = IDLE;
        I: if (rx_data_r == 8'h20) next_state = SPACE1; else next_state = IDLE;
        SPACE1: if (rx_data_r == 8'h6C) next_state = L; else next_state = IDLE;
        L: if (rx_data_r == 8'h69) next_state = I2; else next_state = IDLE;
        I2: if (rx_data_r == 8'h6B) next_state = K; else next_state = IDLE;
        K: if (rx_data_r == 8'h65) next_state = E; else next_state = IDLE;
        E: if (rx_data_r == 8'h20) next_state = SPACE2; else next_state = IDLE;
        SPACE2: if (rx_data_r == 8'h76) next_state = V; else next_state = IDLE;
        V: if (rx_data_r == 8'h65) next_state = E2; else next_state = IDLE;
        E2: if (rx_data_r == 8'h72) next_state = R; else next_state = IDLE;
        R: if (rx_data_r == 8'h69) next_state = I3; else next_state = IDLE;
        I3: if (rx_data_r == 8'h6C) next_state = L2; else next_state = IDLE;
        L2: if (rx_data_r == 8'h6F) next_state = O; else next_state = IDLE;
        O: if (rx_data_r == 8'h67) next_state = G; else next_state = IDLE;
        G: next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

always @(posedge clk or negedge rst) begin
    if (!rst) match <= 0;
    else if (next_state == G) match <= 1;
    else match <= 0;
end



endmodule



//module my_uart_rx
//(
//	input			clk,		//25MHz主时钟
//	input			rst,		//低电平复位信号
//	input			uart_rx,	//接收数据信号

    
//	input			clk_bps,	//clk_bps的高电平为接收或者发送数据位的中间采样点
//	output			bps_start,	//接收到数据后，波特率时钟启动信号置位
//	output 	reg	[14:0] led,

//	    output reg match2,match1
//);






////----------------------------------------------------------------
//reg		uart_rx0,uart_rx1,uart_rx2,uart_rx3;	//接收数据寄存器
//wire	neg_uart_rx;							
 
//always @ (posedge clk or negedge rst)
//begin 
//	if(!rst) 
//    begin
//		uart_rx0 <= 1'b0;
//		uart_rx1 <= 1'b0;
//		uart_rx2 <= 1'b0;
//		uart_rx3 <= 1'b0;
//	end
//	else 
//    begin
//		uart_rx0 <= uart_rx;
//		uart_rx1 <= uart_rx0;
//		uart_rx2 <= uart_rx1;
//		uart_rx3 <= uart_rx2;
//	end
//end

//assign neg_uart_rx = uart_rx3 & ~uart_rx2 ;	//接收到下降沿后neg_uart_rx置高一个时钟周期

////----------------------------------------------------------------
//reg 		bps_start_r;
//reg	[3:0] 	num;	
//reg 		rx_int;		//接收数据中断信号,接收到数据期间始终为高电平

//always @ (posedge clk or negedge rst)
//begin
//	if(!rst)  
//    begin
//		bps_start_r <= 1'bz;
//		rx_int 		<= 1'b0;
//	end
//	else if(neg_uart_rx) 
//    begin	                    //接收到串口接收线uart_rx的下降沿标志信号  
//		bps_start_r <= 1'b1;	//启动串口准备数据接收   
//		rx_int 		<= 1'b1;	//接收数据中断信号使能
//	end
//	else if(num == 4'd9) 
//    begin	                    //接收完有用数据信息
//		bps_start_r <= 1'b0;	//数据接收完毕，释放波特率启动信号
//		rx_int 		<= 1'b0;	//接收数据中断信号关闭
//	end
//end

//assign bps_start = bps_start_r;

////----------------------------------------------------------------
//reg	[7:0]	rx_data_r;		//串口接收数据寄存器
//reg	[7:0]	rx_temp_data;	//当前接收数据寄存器
//reg charge;
//always @ (posedge clk or negedge rst)
//begin
//	if(!rst) 
//    begin
//		rx_temp_data 	<= 8'd0;
//		num 			<= 4'd0;
//		rx_data_r 		<= 8'd0;
//		led				<= 8'b0;
//		charge        <= 0;
//	end
//	else if(rx_int) 
//   begin	                //接收数据处理

//		if(clk_bps) 
//        begin	            	
//			num <= num+1'b1;
            
//			case (num)
//				4'd1: begin
//                    rx_temp_data[0] <= uart_rx;	//锁存第0bit
//                    led[0] <= uart_rx; //将uart_rx赋给led的第0位
//                end
//				4'd2: begin
//                    rx_temp_data[1] <= uart_rx;	//锁存第1bit
//                    led[1] <= uart_rx; //将uart_rx赋给led的第1位
//                end
//				4'd3: begin
//                    rx_temp_data[2] <= uart_rx;	//锁存第2bit
//                    led[2] <= uart_rx; //将uart_rx赋给led的第2位
//                end
//				4'd4: begin
//                    rx_temp_data[3] <= uart_rx;	//锁存第3bit
//                    led[3] <= uart_rx; //将uart_rx赋给led的第3位
//                end
//				4'd5: begin
//                    rx_temp_data[4] <= uart_rx;	//锁存第4bit
//                    led[4] <= uart_rx; //将uart_rx赋给led的第4位
//                end
//				4'd6: begin
//                    rx_temp_data[5] <= uart_rx;	//锁存第5bit
//                    led[5] <= uart_rx; //将uart_rx赋给led的第5位
//                end  
//				4'd7: begin
//                    rx_temp_data[6] <= uart_rx;	//锁存第6bit
//                    led[6] <= uart_rx; //将uart_rx赋给led的第6位
//                end 
//				4'd8: begin
//                    rx_temp_data[7] <= uart_rx;	//锁存第7bit
//                    led[7] <= uart_rx; //将uart_rx赋给led的第7位
//                end 
//				default: ;
//			endcase
//		end
//		else if(num == 4'd9) 
//        begin		
//            charge      <=1;
//			num			<= 4'd0;			//接收到STOP位后结束,num清零
//			rx_data_r	<= rx_temp_data;	//把数据锁存到数据寄存器rx_data中
			
////			led = rx_temp_data;
////			led <= rx_data_r;
//		end else charge <= 0;
//	end
//end



//assign rx_data = rx_data_r;	






//parameter IDLE = 4'b0000, I = 4'b0001, SPACE1 = 4'b0010, L = 4'b0011, 
//           I2 = 4'b0100, K = 4'b0101, E = 4'b0110, SPACE2 = 4'b0111, 
//           F = 4'b1000, P = 4'b1001, G = 4'b1010, A = 4'b1011, V = 4'b1100, R = 4'b1101, O = 4'b1110, L2 = 4'b1111;
//reg [3:0] state, next_state;

//initial next_state = IDLE;
//always @( posedge charge or negedge rst) begin
//    if (!rst) state <= IDLE;
//    else 
//    case (next_state)
//        IDLE: if (rx_data_r == 8'h69) next_state = I; else next_state = IDLE;
//        I: if (rx_data_r == 8'h20) next_state = SPACE1; else next_state = IDLE;
//        SPACE1: if (rx_data_r == 8'h6C) next_state = L; else next_state = IDLE;
//        L: if (rx_data_r == 8'h69) next_state = I2; else next_state = IDLE;
//        I2: if (rx_data_r == 8'h6B) next_state = K; else next_state = IDLE;
//        K: if (rx_data_r == 8'h65) next_state = E; else next_state = IDLE;
//        E: if (rx_data_r == 8'h20) next_state = SPACE2; else next_state = IDLE;
//        SPACE2: if (rx_data_r == 8'h46) next_state = F; else if (rx_data_r == 8'h76) next_state = V; else next_state = IDLE;
//        F: if (rx_data_r == 8'h50) next_state = P; else next_state = IDLE;
//        P: if (rx_data_r == 8'h47) next_state = G; else next_state = IDLE;
//        G: if (rx_data_r == 8'h41) next_state = A; else next_state = IDLE;
//        A: next_state = IDLE;
//        V: if (rx_data_r == 8'h65) next_state = E; else next_state = IDLE;
//        E: if (rx_data_r == 8'h72) next_state = R; else next_state = IDLE;
//        R: if (rx_data_r == 8'h69) next_state = I; else next_state = IDLE;
//        I: if (rx_data_r == 8'h6C) next_state = L2; else next_state = IDLE;
//        L2: if (rx_data_r == 8'h6F) next_state = O; else next_state = IDLE;
//        O: if (rx_data_r == 8'h67) next_state = G; else next_state = IDLE;
//        default: next_state = IDLE;
//    endcase
//end

//always @(posedge clk or negedge rst) begin
//    if (!rst)begin
//     match1 <= 0;
//     match2 <= 0;
//     end
//    else if ( next_state == A) match1 <= 1;
//     else if (next_state == G ) match2 <= 1;

//end





//endmodule
