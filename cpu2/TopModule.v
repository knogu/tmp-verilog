module TopModule(
	//////////// CLOCK //////////
	input 		          		CLK1,
	input 		          		CLK2,
	//////////// SEG7 //////////
	output		     [7:0]		HEX0,
	output		     [7:0]		HEX1,
	output		     [7:0]		HEX2,
	output		     [7:0]		HEX3,
	output		     [7:0]		HEX4,
	output		     [7:0]		HEX5,
	//////////// Push Button //////////
	input 		     [1:0]		BTN,
	//////////// LED //////////
	output		     [9:0]		LED,
	//////////// SW //////////
	input 		     [9:0]		SW

	);

    wire [31:0] a0;
	m_proc3 m_cpu(CKL1, a0);
    assign HEX0=a0[7:0];
    assign HEX1=a0[15:8];
    assign HEX2=a0[23:16];
    assign HEX3=a0[31:24];
	
endmodule

