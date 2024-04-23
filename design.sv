module top(input pclk,
          input presetn,
           input [31:0] pwdata,
           input [31:0] paddr,
          input psel,
          input penable,
          input pwrite,
           output reg [31:0] prdata);
  
  reg [3:0] cntrl;
  reg [31:0] reg1;
  reg [31:0] reg2;
  reg [31:0] reg3;
  reg [31:0] reg4;
  reg [31:0] temp;
  
  always @(posedge pclk)
    begin
      if (!presetn)
        begin
          cntrl <= 4'b0;
          reg1 <= 32'b0;
          reg2 <= 32'b0;
          reg3 <= 32'b0;
          reg4 <= 32'b0;
          temp <= 32'b0;
        end
      else
        begin
          if (pwrite && penable && psel)
            begin
              case(paddr)
                  'h0: cntrl <= pwdata;
                  'h4: reg1 <= pwdata;
                  'h8: reg2 <= pwdata;
                  'hc: reg3 <= pwdata;
                  'h10: reg4 <= pwdata;
              endcase
            end
          else
            begin
              case(paddr)
                'h0: temp <= cntrl;
                'h4: temp <= reg1;
                'h8: temp <= reg2;
                'hc: temp <= reg3;
                'h10: temp <= reg4;
                default: temp = 32'h00000000;
              endcase
            end
        end
    end
  assign prdata = temp;
endmodule


interface top_if;
  logic [31:0] pwdata,prdata,paddr;
  logic pwrite, penable, psel;
  logic presetn, pclk;
endinterface