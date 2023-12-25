
module my_uart_rx1
(
	input			clk,		//25MHz��ʱ��
	input			rst,		//�͵�ƽ��λ�ź�
	input			uart_rx,	//���������ź�
	output	[7:0]	rx_data,	//�������ݼĴ���������ֱ����һ����������
    
	input			clk_bps,	//clk_bps�ĸߵ�ƽΪ���ջ��߷�������λ���м������
	output			bps_start,	//���յ����ݺ󣬲�����ʱ�������ź���λ
	output 	reg	[14:0] led,
	 output reg [3:0] next_state,
	    output reg match
);






//----------------------------------------------------------------
reg		uart_rx0,uart_rx1,uart_rx2,uart_rx3;	//�������ݼĴ���
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

assign neg_uart_rx = uart_rx3 & ~uart_rx2 ;	//���յ��½��غ�neg_uart_rx�ø�һ��ʱ������

//----------------------------------------------------------------
reg 		bps_start_r;
reg	[3:0] 	num;	
reg 		rx_int;		//���������ж��ź�,���յ������ڼ�ʼ��Ϊ�ߵ�ƽ

always @ (posedge clk or negedge rst)
begin
	if(!rst)  
    begin
		bps_start_r <= 1'bz;
		rx_int 		<= 1'b0;
	end
	else if(neg_uart_rx) 
    begin	                    //���յ����ڽ�����uart_rx���½��ر�־�ź�  
		bps_start_r <= 1'b1;	//��������׼�����ݽ���   
		rx_int 		<= 1'b1;	//���������ж��ź�ʹ��
	end
	else if(num == 4'd9) 
    begin	                    //����������������Ϣ
		bps_start_r <= 1'b0;	//���ݽ�����ϣ��ͷŲ����������ź�
		rx_int 		<= 1'b0;	//���������ж��źŹر�
	end
end

assign bps_start = bps_start_r;

//----------------------------------------------------------------
reg	[7:0]	rx_data_r;		//���ڽ������ݼĴ���
reg	[7:0]	rx_temp_data;	//��ǰ�������ݼĴ���
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
   begin	                //�������ݴ���

		if(clk_bps) 
        begin	            	
			num <= num+1'b1;
            
			case (num)
				4'd1: begin
                    rx_temp_data[0] <= uart_rx;	//�����0bit
                    led[0] <= uart_rx; //��uart_rx����led�ĵ�0λ
                end
				4'd2: begin
                    rx_temp_data[1] <= uart_rx;	//�����1bit
                    led[1] <= uart_rx; //��uart_rx����led�ĵ�1λ
                end
				4'd3: begin
                    rx_temp_data[2] <= uart_rx;	//�����2bit
                    led[2] <= uart_rx; //��uart_rx����led�ĵ�2λ
                end
				4'd4: begin
                    rx_temp_data[3] <= uart_rx;	//�����3bit
                    led[3] <= uart_rx; //��uart_rx����led�ĵ�3λ
                end
				4'd5: begin
                    rx_temp_data[4] <= uart_rx;	//�����4bit
                    led[4] <= uart_rx; //��uart_rx����led�ĵ�4λ
                end
				4'd6: begin
                    rx_temp_data[5] <= uart_rx;	//�����5bit
                    led[5] <= uart_rx; //��uart_rx����led�ĵ�5λ
                end  
				4'd7: begin
                    rx_temp_data[6] <= uart_rx;	//�����6bit
                    led[6] <= uart_rx; //��uart_rx����led�ĵ�6λ
                end 
				4'd8: begin
                    rx_temp_data[7] <= uart_rx;	//�����7bit
                    led[7] <= uart_rx; //��uart_rx����led�ĵ�7λ
                end 
				default: ;
			endcase
		end
		else if(num == 4'd9) 
        begin		
            charge      <=1;
			num			<= 4'd0;			//���յ�STOPλ�����,num����
			rx_data_r	<= rx_temp_data;	//���������浽���ݼĴ���rx_data��
			
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
//	input			clk,		//25MHz��ʱ��
//	input			rst,		//�͵�ƽ��λ�ź�
//	input			uart_rx,	//���������ź�

    
//	input			clk_bps,	//clk_bps�ĸߵ�ƽΪ���ջ��߷�������λ���м������
//	output			bps_start,	//���յ����ݺ󣬲�����ʱ�������ź���λ
//	output 	reg	[14:0] led,

//	    output reg match2,match1
//);






////----------------------------------------------------------------
//reg		uart_rx0,uart_rx1,uart_rx2,uart_rx3;	//�������ݼĴ���
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

//assign neg_uart_rx = uart_rx3 & ~uart_rx2 ;	//���յ��½��غ�neg_uart_rx�ø�һ��ʱ������

////----------------------------------------------------------------
//reg 		bps_start_r;
//reg	[3:0] 	num;	
//reg 		rx_int;		//���������ж��ź�,���յ������ڼ�ʼ��Ϊ�ߵ�ƽ

//always @ (posedge clk or negedge rst)
//begin
//	if(!rst)  
//    begin
//		bps_start_r <= 1'bz;
//		rx_int 		<= 1'b0;
//	end
//	else if(neg_uart_rx) 
//    begin	                    //���յ����ڽ�����uart_rx���½��ر�־�ź�  
//		bps_start_r <= 1'b1;	//��������׼�����ݽ���   
//		rx_int 		<= 1'b1;	//���������ж��ź�ʹ��
//	end
//	else if(num == 4'd9) 
//    begin	                    //����������������Ϣ
//		bps_start_r <= 1'b0;	//���ݽ�����ϣ��ͷŲ����������ź�
//		rx_int 		<= 1'b0;	//���������ж��źŹر�
//	end
//end

//assign bps_start = bps_start_r;

////----------------------------------------------------------------
//reg	[7:0]	rx_data_r;		//���ڽ������ݼĴ���
//reg	[7:0]	rx_temp_data;	//��ǰ�������ݼĴ���
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
//   begin	                //�������ݴ���

//		if(clk_bps) 
//        begin	            	
//			num <= num+1'b1;
            
//			case (num)
//				4'd1: begin
//                    rx_temp_data[0] <= uart_rx;	//�����0bit
//                    led[0] <= uart_rx; //��uart_rx����led�ĵ�0λ
//                end
//				4'd2: begin
//                    rx_temp_data[1] <= uart_rx;	//�����1bit
//                    led[1] <= uart_rx; //��uart_rx����led�ĵ�1λ
//                end
//				4'd3: begin
//                    rx_temp_data[2] <= uart_rx;	//�����2bit
//                    led[2] <= uart_rx; //��uart_rx����led�ĵ�2λ
//                end
//				4'd4: begin
//                    rx_temp_data[3] <= uart_rx;	//�����3bit
//                    led[3] <= uart_rx; //��uart_rx����led�ĵ�3λ
//                end
//				4'd5: begin
//                    rx_temp_data[4] <= uart_rx;	//�����4bit
//                    led[4] <= uart_rx; //��uart_rx����led�ĵ�4λ
//                end
//				4'd6: begin
//                    rx_temp_data[5] <= uart_rx;	//�����5bit
//                    led[5] <= uart_rx; //��uart_rx����led�ĵ�5λ
//                end  
//				4'd7: begin
//                    rx_temp_data[6] <= uart_rx;	//�����6bit
//                    led[6] <= uart_rx; //��uart_rx����led�ĵ�6λ
//                end 
//				4'd8: begin
//                    rx_temp_data[7] <= uart_rx;	//�����7bit
//                    led[7] <= uart_rx; //��uart_rx����led�ĵ�7λ
//                end 
//				default: ;
//			endcase
//		end
//		else if(num == 4'd9) 
//        begin		
//            charge      <=1;
//			num			<= 4'd0;			//���յ�STOPλ�����,num����
//			rx_data_r	<= rx_temp_data;	//���������浽���ݼĴ���rx_data��
			
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
