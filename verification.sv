`include "uvm_macros.svh"
import uvm_pkg::*;

class transaction extends uvm_sequence_item;
  `uvm_object_utils(transaction)
  
  bit [31:0] pwdata;
  bit [31:0] paddr;
  bit pwrite,psel,penable;
  bit [31:0] prdata;
  
  function new(string name = "trans");
    super.new(name);
  endfunction
endclass

/////////////////////////////////////////////////////////////////////////////////

class driver extends uvm_driver #(transaction);
  `uvm_component_utils(driver)
  
  transaction tr;
  virtual top_if vif;
  
  function new(input string path = "driver", uvm_component parent = null);
    super.new(path,parent);
  endfunction
 
  virtual function void build_phase(uvm_phase phase);
  	super.build_phase(phase);
    tr = transaction::type_id::create("trans");
    if(!uvm_config_db#(virtual top_if)::get(this,"","vif",vif))
      `uvm_error("drv","Unable to access Interface");
  endfunction
  
  task reset_dut();
    vif.presetn <= 1'b0;
    vif.penable <= 1'b0;
    vif.pwrite <= 1'b0;
    repeat (2) @(posedge vif.pclk);
    vif.presetn <= 1'b1;
  endtask
  
  virtual task run_phase(uvm_phase phase);
    reset_dut();
    forever begin
      seq_item_port.get_next_item(tr);
      if (tr.pwrite)
        begin
          vif.pwrite <= tr.pwrite;
          vif.psel <= 1'b1;
          vif.pwdata <= tr.pwdata;
          vif.paddr <= tr.paddr;
          @(posedge vif.pclk);
          vif.penable <= 1'b1;
        end
      else
        begin 
          vif.pwrite <= tr.pwrite;
          vif.psel <= 1'b1;
          vif.paddr <= tr.paddr;
          @(posedge vif.pclk);
          vif.penable <= 1'b1;
        end
      @(posedge vif.pclk);
      `uvm_info("drv", $sformatf("pwrite = %0d, pwdata = %0d, paddr = %0d, prdata = %0d", tr.pwrite, tr.pwdata, tr.paddr, tr.prdata), UVM_NONE)
      @(posedge vif.pclk);
      seq_item_port.item_done();
    end
  endtask
endclass

////////////////////////////////////////////////////////////////////////////////

class monitor extends uvm_monitor;
  `uvm_component_utils(monitor)
  
  transaction tr;
  virtual top_if vif;
  uvm_analysis_port #(transaction) mon_ap;
  
  function  new(string name = "mon", uvm_component parent = null);
    super.new(name, parent);
  endfunction
  
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    tr = transaction::type_id::create("trans");
    mon_ap = new("mon_ap", this);
    if (!uvm_config_db #(virtual top_if)::get(this,"","vif",vif))
      `uvm_error("mon", "cannot access the interface");
  endfunction
  
  virtual task run_phase(uvm_phase phase);
    @(posedge vif.presetn);
    forever begin
          repeat (3) @(posedge vif.pclk);
            tr.pwrite = vif.pwrite;
            tr.penable = vif.penable;
            tr.psel = vif.psel;
            tr.pwdata = vif.pwdata;
            tr.prdata = vif.prdata;
            tr.paddr = vif.paddr;
          `uvm_info("mon", $sformatf("pwrite = %0d, paddr = %0d pwdata = %0d prdata = %0d", tr.pwrite, tr.paddr, tr.pwdata, tr.prdata), UVM_NONE)
          mon_ap.write(tr);
        end
  endtask
endclass

////////////////////////////////////////////////////////////////////////////////

class scoreboard extends uvm_scoreboard;
  `uvm_component_utils(scoreboard)
  
  uvm_analysis_imp #(transaction, scoreboard) recv;
  reg [3:0] cntrlt;
  reg [31:0] reg1t;
  reg [31:0] reg2t;
  reg [31:0] reg3t;
  reg [31:0] reg4t;
  
  function new(string path = "sb", uvm_component parent = null);
    super.new(path, parent);
  endfunction
  
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    recv = new("recv", this);
  endfunction
  
  virtual function void write(transaction tr);
    calculate(tr);
  endfunction
  
  function void calculate(transaction tr);
    if (tr.pwrite) begin
      case (tr.paddr)
          'h0: cntrlt <= tr.pwdata;
          'h4: reg1t <= tr.pwdata;
          'h8: reg2t <= tr.pwdata;
          'hc: reg3t <= tr.pwdata;
          'h10: reg4t <= tr.pwdata;
      endcase
    end
    else
      begin
        $display("-----------------------------------------------");
        case(tr.paddr)
    'h0:
        begin
            if (tr.prdata == cntrlt)
                $display("Control register test passed");
            else
                $display("Control register test failed");
        end
    'h4:
        begin
            if (tr.prdata == reg1t)
                $display("Register 1 test passed");
            else
                $display("Register 1 test failed");
        end
    'h8:
        begin
            if (tr.prdata == reg2t)
                $display("Register 2 test passed");
            else
                $display("Register 2 test failed");
        end
    'hc:
        begin
            if (tr.prdata == reg3t)
                $display("Register 3 test passed");
            else
                $display("Register 3 test failed");
        end
    'h10:
        begin
            if (tr.prdata == reg4t)
                $display("Register 4 test passed");
            else
                $display("Register 4 test failed");
        end
    default:
        $display("Unknown register address");
endcase
        $display("-------------------------------------------");
      end
  endfunction
endclass

////////////////////////////////////////////////////////////////////////////////

class agent extends uvm_agent;
  `uvm_component_utils(agent)
  
  driver d;
  monitor m;
  uvm_sequencer #(transaction) seqr;
  
  function new(string path = "ag", uvm_component parent = null);
    super.new(path,parent);
  endfunction
  
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    d = driver::type_id::create("drv", this);
    m = monitor::type_id::create("mon", this);
    seqr = uvm_sequencer #(transaction)::type_id::create("seqr", this);
  endfunction
  
  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    d.seq_item_port.connect(seqr.seq_item_export);
  endfunction
endclass

///////////////////////////////////////////////////////////////////////////////

// reg model for dut

class cntrl_reg extends uvm_reg;
  `uvm_object_utils(cntrl_reg)
  
  rand uvm_reg_field cntrl;
  
  covergroup cntrl_cov;
    
    option.per_instance = 1;
    
    coverpoint cntrl.value[3:0]
    {
      bins low = {[0:10]};
      bins high = {[11:20]};
    }
  endgroup
  
  function new(string name = "cntrl_reg");
    super.new(name, 4, UVM_CVR_FIELD_VALS);
    
    if(has_coverage(UVM_CVR_FIELD_VALS))
      cntrl_cov = new();
  endfunction
  
  virtual function void sample(uvm_reg_data_t data,
                              uvm_reg_data_t byte_en,
                              bit is_read,
                              uvm_reg_map map);
    cntrl_cov.sample();
    
  endfunction
  
  virtual function void sample_values();
    super.sample_values();
    cntrl_cov.sample();
  endfunction
  
  function void build();
    cntrl = uvm_reg_field::type_id::create("cntrl");
    cntrl.configure(this, 4, 0, "RW", 0, 4'h0 , 1, 1, 1);
  endfunction
  
endclass

//////////////////////////////////////////////////////////////////////////////

class reg1_reg extends uvm_reg;
    `uvm_object_utils(reg1_reg)
    
    rand uvm_reg_field reg1;
  parameter binsize  = (2**32)/8;
  
    covergroup reg1_cov;
        option.per_instance = 1;
      
        
        coverpoint reg1.value[31:0]
        {
            bins low = {[0:10]};
      		bins high = {[11:20]};
        }
    endgroup
  
    function new(string name = "reg1_reg");
        super.new(name, 32, UVM_CVR_FIELD_VALS);
        
        if(has_coverage(UVM_CVR_FIELD_VALS))
            reg1_cov = new();
    endfunction
  
    virtual function void sample(uvm_reg_data_t data,
                                  uvm_reg_data_t byte_en,
                                  bit is_read,
                                  uvm_reg_map map);
        reg1_cov.sample();
    endfunction
  
    virtual function void sample_values();
        super.sample_values();
        reg1_cov.sample();
    endfunction
    
    function build();
        reg1 = uvm_reg_field::type_id::create("reg1");
        reg1.configure(this, 32, 0, "RW", 0, 32'h0, 1, 1, 1);
    endfunction
endclass


///////////////////////////////////////////////////////////////////////////////

class reg2_reg extends uvm_reg;
    `uvm_object_utils(reg2_reg)
    
    rand uvm_reg_field reg2;
  parameter binsize  = (2**32)/8;
  
    covergroup reg2_cov;
        option.per_instance = 1;
        
        coverpoint reg2.value[31:0]
        {
            bins low = {[0:10]};
      		bins high = {[11:20]};
        }
    endgroup
  
    function new(string name = "reg2_reg");
        super.new(name, 32, UVM_CVR_FIELD_VALS);
        
        if(has_coverage(UVM_CVR_FIELD_VALS))
            reg2_cov = new();
    endfunction
  
    virtual function void sample(uvm_reg_data_t data,
                                  uvm_reg_data_t byte_en,
                                  bit is_read,
                                  uvm_reg_map map);
        reg2_cov.sample();
    endfunction
  
    virtual function void sample_values();
        super.sample_values();
        reg2_cov.sample();
    endfunction
    
    function build();
        reg2 = uvm_reg_field::type_id::create("reg2");
        reg2.configure(this, 32, 0, "RW", 0, 32'h0, 1, 1, 1);
    endfunction
endclass


////////////////////////////////////////////////////////////////////////////////

class reg3_reg extends uvm_reg;
    `uvm_object_utils(reg3_reg)
    
    rand uvm_reg_field reg3;
  parameter binsize = 2**32/8;
  
    covergroup reg3_cov;
        option.per_instance = 1;
        
        
        coverpoint reg3.value[31:0]
        {
            bins low = {[0:10]};
      		bins high = {[11:20]};
            
        }
    endgroup
  
    function new(string name = "reg3_reg");
        super.new(name, 32, UVM_CVR_FIELD_VALS);
        
        if(has_coverage(UVM_CVR_FIELD_VALS))
            reg3_cov = new();
    endfunction
  
    virtual function void sample(uvm_reg_data_t data,
                                  uvm_reg_data_t byte_en,
                                  bit is_read,
                                  uvm_reg_map map);
        reg3_cov.sample();
    endfunction
  
    virtual function void sample_values();
        super.sample_values();
        reg3_cov.sample();
    endfunction
    
    function build();
        reg3 = uvm_reg_field::type_id::create("reg3");
        reg3.configure(this, 32, 0, "RW", 0, 32'h0, 1, 1, 1);
    endfunction
endclass


/////////////////////////////////////////////////////////////////////////////


class reg4_reg extends uvm_reg;
    `uvm_object_utils(reg4_reg)
    
    rand uvm_reg_field reg4;
  parameter binsize = 2**32/8;
    
    covergroup reg4_cov;
    option.per_instance = 1;
      
    
    coverpoint reg4.value[31:0]
    {
      		bins low = {[0:10]};
      		bins high = {[11:20]};
    }
  endgroup
  
  function new(string name = "reg4_reg");
    super.new(name, 32, UVM_CVR_FIELD_VALS);
    
    if(has_coverage(UVM_CVR_FIELD_VALS))
      reg4_cov = new();
    
  endfunction
  
  virtual function void sample(uvm_reg_data_t data,
                              uvm_reg_data_t byte_en,
                              bit is_read,
                              uvm_reg_map map);
    reg4_cov.sample();
    
  endfunction
  
  virtual function void sample_values();
    super.sample_values();
    reg4_cov.sample();
  endfunction
    
    function build();
        reg4 = uvm_reg_field::type_id::create("reg4");
        reg4.configure(this, 32, 0, "RW", 0, 32'h0, 1, 1, 1);
    endfunction
endclass

////////////////////////////////////////////////////////////////////////////

class top_reg_block extends uvm_reg_block;
  `uvm_object_utils(top_reg_block)
  
  cntrl_reg c;
  reg1_reg r1;
  reg2_reg r2;
  reg3_reg r3;
  reg4_reg r4;
  
  function new(string name = "top_reg_block");
    super.new(name, build_coverage(UVM_CVR_FIELD_VALS));
  endfunction
  
  function void build();
    
    uvm_reg::include_coverage("*", UVM_CVR_ALL);
    
    // For cntrl_reg
    c = cntrl_reg::type_id::create("cntrl_reg");
    c.build();
    c.configure(this, null);
    c.set_coverage(UVM_CVR_FIELD_VALS); //////enabling coverage for specific reg instance   

    // For reg1_reg
    r1 = reg1_reg::type_id::create("reg1_reg");
    r1.build();
    r1.configure(this, null);
    r1.set_coverage(UVM_CVR_FIELD_VALS);

    // For reg2_reg
    r2 = reg2_reg::type_id::create("reg2_reg");
    r2.build();
    r2.configure(this, null);
    r2.set_coverage(UVM_CVR_FIELD_VALS);

    // For reg3_reg
    r3 = reg3_reg::type_id::create("reg3_reg");
    r3.build();
    r3.configure(this, null);
    r3.set_coverage(UVM_CVR_FIELD_VALS);

    // For reg4_reg
    r4 = reg4_reg::type_id::create("reg4_reg");
    r4.build();
    r4.configure(this, null);
    r4.set_coverage(UVM_CVR_FIELD_VALS);
    
    default_map = create_map("default_map", 0, 4, UVM_LITTLE_ENDIAN, 0); // name, base, nBytes
    default_map.add_reg(c, 'h0, "RW" ); // name , offset address and access type

    // For reg1_reg
    default_map.add_reg(r1, 'h4, "RW");

    // For reg2_reg
    default_map.add_reg(r2, 'h8, "RW");

    // For reg3_reg
    default_map.add_reg(r3, 'hc, "RW");

    // For reg4_reg
    default_map.add_reg(r4, 'h10, "RW");
    
    lock_model();
  endfunction
endclass

////////////////////////////////////////////////////////////////////////////////

class top_reg_seq extends uvm_sequence;
  `uvm_object_utils(top_reg_seq)
  
  top_reg_block regmodel;
  
  function new(string path = "top_reg_block");
    super.new(path);
  endfunction
  
  task body;
    uvm_status_e   status;
    reg [31:0] value, rvalue, data;
    int i;
    
    for (i = 0; i <= 4; i++) begin
    // Generate random data for each register
      data = $urandom_range(0,20);
    
    // Perform write and read sequences for each register
    case (i)
        0:
            begin
              regmodel.c.write(status, data[3:0]);
              regmodel.c.read(status, rvalue);
            end
        1:
            begin
                regmodel.r1.write(status, data);
                regmodel.r1.read(status, rvalue);
            end
        2:
            begin
                regmodel.r2.write(status, data);
                regmodel.r2.read(status, rvalue);
            end
        3:
            begin
                regmodel.r3.write(status, data);
                regmodel.r3.read(status, rvalue);
            end
        4:
            begin
                regmodel.r4.write(status, data);
                regmodel.r4.read(status, rvalue);
            end
    endcase
end

    
  endtask
  
endclass

//////////////////////////////////////////////////////////////////////////////

class adapter extends uvm_reg_adapter;
  `uvm_object_utils(adapter)
  
  function new (string name = "top_adapter");
      super.new (name);
   endfunction
  
  //reg2bus method
  
  function uvm_sequence_item reg2bus(const ref uvm_reg_bus_op rw);
    transaction tr;
    tr = transaction::type_id::create("trans");
    tr.pwrite = (rw.kind == UVM_WRITE);
    tr.paddr = rw.addr;
    if (tr.pwrite) tr.pwdata = rw.data;
    return tr;
  endfunction
  
  //bus2reg method
  
  function void bus2reg(uvm_sequence_item bus_item, ref uvm_reg_bus_op rw);
    
    transaction tr;
    assert($cast(tr,bus_item));
    
    rw.kind = (tr.pwrite == 1) ? UVM_WRITE:UVM_READ;
    rw.data = tr.prdata;
    rw.addr = tr.paddr;
    rw.status = UVM_IS_OK;
  endfunction
  
endclass

//////////////////////////////////////////////////////////////////////////////

class env extends uvm_env;
  `uvm_component_utils(env)
  
  scoreboard sb;
  agent ag;
  top_reg_block regmodel;
  adapter a;
  uvm_reg_predictor #(transaction) p;
  
  function new (string path = "env", uvm_component parent = null);
    super.new(path, parent);
  endfunction
  
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    sb = scoreboard::type_id::create("sb", this);
    ag = agent::type_id::create("ag", this);
    regmodel = top_reg_block::type_id::create("trp", this);
    regmodel.build();
    p = uvm_reg_predictor #(transaction)::type_id::create("p", this);
    a = adapter::type_id::create("a",, get_full_name());
  endfunction
  
  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    
    // scoreboard and monitorconnection
    
    ag.m.mon_ap.connect(sb.recv);
    
    // adapter and sequencer connection
    regmodel.default_map.set_sequencer(.sequencer(ag.seqr), .adapter(a));
    regmodel.default_map.set_base_addr(0);
    
    //predictor connection
    
    p.map = regmodel.default_map;
    p.adapter = a;
    ag.m.mon_ap.connect(p.bus_in);
  endfunction
endclass

///////////////////////////////////////////////////////////////////////////////

class test extends uvm_test;
  `uvm_component_utils(test)
  
  env e;
  top_reg_seq s;
  
  function new(string path = "test", uvm_component parent = null);
    super.new(path,parent);
  endfunction
  
  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    e = env::type_id::create("e", this);
    s = top_reg_seq::type_id::create("trs");
  endfunction
  
  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    s.regmodel = e.regmodel;
    s.start(e.ag.seqr);
    phase.drop_objection(this);
    phase.phase_done.set_drain_time(this, 200);
  endtask
endclass

/////////////////////////////////////////////////////////////////

module tb;
  top_if vif();
  
  top dut (.pclk(vif.pclk),
    		.presetn(vif.presetn),
    		.pwdata(vif.pwdata),
    		.pwrite(vif.pwrite),
    		.penable(vif.penable),
    		.psel(vif.psel),
           .prdata(vif.prdata),
           .paddr(vif.paddr));
  
  initial 
    begin
    vif.pclk = 0;
    end
  
  always #10 vif.pclk = ~vif.pclk;
  
  initial begin
    uvm_config_db #(virtual top_if)::set(null, "*", "vif", vif);
    run_test("test");
  end
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
  end
endmodule