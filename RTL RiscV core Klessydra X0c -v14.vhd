---------------------------------------------------------------------------------------------------
-- Klessydra T0m43 core -- 
-- RI5CY pinout, RISCV core, RV32I Base Integer Instruction Set, 4 pipeline stages F/D/E/W, 
-- in order execution, stalled branches, supports variable latency program memory. 
-- Supports interleaved multithreading.
-- RISCV compliant exception and interrupt handling. Only thread 0 can be interrupted.
-- RI5CY irq/exception table is fully supported by software runtime system.
-- Contributors: Gianmarco Cerutti, Abdallah Cheikh, Ivan Matraxia, Stefano Sordillo, 
--               Francesco Vigli, M. Olivieri
-- last update: 2017-09-09
---------------------------------------------------------------------------------------------------


-- ieee packages ------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;
use std.textio.all;

-- local packages ------------
use work.riscv_klessydra.all;

-- core pinout --------------------
entity riscv_core is
  generic (
    N_EXT_PERF_COUNTERS : integer := 0;   -- ignored in Klessydra
    INSTR_RDATA_WIDTH   : integer := 32;  -- ignored in Klessydra
    N_HWLP              : integer := 2;  -- ignored in Klessydra
    N_HWLP_BITS         : integer := 4  -- ignored in Klessydra
    );
  port (
    -- clock, reset active low, test enable
    clk_i               : in  std_logic;
    clock_en_i          : in  std_logic;
    rst_ni              : in  std_logic;
    test_en_i           : in  std_logic;
    -- initialization signals 
    boot_addr_i         : in  std_logic_vector(31 downto 0);
    core_id_i           : in  std_logic_vector(3 downto 0);
    cluster_id_i        : in  std_logic_vector(5 downto 0);
    -- program memory interface
    instr_req_o         : out std_logic;
    instr_gnt_i         : in  std_logic;
    instr_rvalid_i      : in  std_logic;
    instr_addr_o        : out std_logic_vector(31 downto 0);
    instr_rdata_i       : in  std_logic_vector(31 downto 0);  
    -- data memory interface
    data_req_o          : out std_logic;
    data_gnt_i          : in  std_logic;
    data_rvalid_i       : in  std_logic;
    data_we_o           : out std_logic;
    data_be_o           : out std_logic_vector(3 downto 0);
    data_addr_o         : out std_logic_vector(31 downto 0);
    data_wdata_o        : out std_logic_vector(31 downto 0);
    data_rdata_i        : in  std_logic_vector(31 downto 0);
    data_err_i          : in  std_logic;
    -- interrupt request interface
    irq_i               : in  std_logic;
    irq_id_i            : in  std_logic_vector(4 downto 0);
    irq_ack_o           : out std_logic;
    irq_id_o            : out std_logic_vector(4 downto 0);
    irq_sec_i           : in  std_logic; -- unused in Pulpino
    sec_lvl_o           : out std_logic; -- unused in Pulpino
    -- debug interface
    debug_req_i         : in  std_logic;
    debug_gnt_o         : out std_logic;
    debug_rvalid_o      : out std_logic;
    debug_addr_i        : in  std_logic_vector(14 downto 0);
    debug_we_i          : in  std_logic;
    debug_wdata_i       : in  std_logic_vector(31 downto 0);
    debug_rdata_o       : out std_logic_vector(31 downto 0);
    debug_halted_o      : out std_logic;
    debug_halt_i        : in  std_logic;
    debug_resume_i      : in  std_logic;
    -- miscellanous control signals
    fetch_enable_i      : in  std_logic;
    core_busy_o         : out std_logic;  -- presently unused
    ext_perf_counters_i : in  std_logic_vector(N_EXT_PERF_COUNTERS to 1)
    );
end entity riscv_core;  ------------------------------------------


--RI5CY "Klessydra" X0 core implementation -----------------------
architecture Klessydra_X0 of riscv_core is

  -- state signals
  type fsm_IF_states is (normal, waiting);
  type fsm_IE_states is (sleep, reset, normal, data_valid_waiting, data_grant_waiting,
                         csr_instr_wait_state, debug, first_boot);

  signal state_IF, nextstate_IF : fsm_IF_states;
  signal state_IE, nextstate_IE : fsm_IE_states;
  signal instr_rvalid_state     : std_logic;
  signal busy_ID                : std_logic;
  signal busy_IE                : std_logic;

  -- Control Status Register (CSR) signals 
  signal MSTATUS  : replicated_32b_reg;
  signal MEPC     : replicated_32b_reg;
  signal MCAUSE   : replicated_32b_reg;
  signal PCCRs    : replicated_32b_reg;  -- still not implemented
  signal PCER     : replicated_32b_reg;  -- still not implemented
  signal PCMR     : replicated_32b_reg;  -- still not implemented
  signal MESTATUS : replicated_32b_reg;
  signal MCPUID   : replicated_32b_reg;
  signal MIMPID   : replicated_32b_reg;
  signal MHARTID  : replicated_32b_reg;
  signal MIP      : replicated_32b_reg;
  signal MTVEC    : replicated_32b_reg;
  signal MIRQ     : replicated_32b_reg;  -- extension, maps external irqs
  signal MBADADDR : replicated_32b_reg;  -- misaligned address containers

  signal MCYCLE          : replicated_32b_reg;
  signal MINSTRET        : replicated_32b_reg;
  signal MHPMCOUNTER3    : replicated_32b_reg;
  --signal MHPMCOUNTER4 : replicated_32b_reg; 
  --signal MHPMCOUNTER5 : replicated_32b_reg; 
  signal MHPMCOUNTER6    : replicated_32b_reg;
  signal MHPMCOUNTER7    : replicated_32b_reg;
  signal MHPMCOUNTER8    : replicated_32b_reg;
  signal MHPMCOUNTER9    : replicated_32b_reg;
  signal MHPMCOUNTER10   : replicated_32b_reg;
  signal MHPMCOUNTER11   : replicated_32b_reg;
  signal MCYCLEH         : replicated_32b_reg;
  signal MINSTRETH       : replicated_32b_reg;
  signal MHPMEVENT3      : replicated_bit;
  --signal MHPMEVENT4           : replicated_bit;        
  --signal MHPMEVENT5           : replicated_bit;        
  signal MHPMEVENT6      : replicated_bit;
  signal MHPMEVENT7      : replicated_bit;
  signal MHPMEVENT8      : replicated_bit;
  signal MHPMEVENT9      : replicated_bit;
  signal MHPMEVENT10     : replicated_bit;
  signal MHPMEVENT11     : replicated_bit;
  signal irq_pending     : replicated_bit;
  signal except_pc_vec_o : std_logic_vector(31 downto 0);
  -- auxiliary irq fixed connection signals
  signal MIP_7           : std_logic;
  signal MIP_11          : std_logic;

  -- Memory fault signals
  signal load_err, store_err : std_logic;

  -- Interface signals from EXEC unit to CSR management unit
  signal csr_instr_req       : std_logic;
  signal csr_instr_done      : std_logic;
  signal csr_access_denied_o : std_logic;
  signal csr_wdata_i         : std_logic_vector (31 downto 0);
  signal csr_op_i            : std_logic_vector (2 downto 0);
  signal csr_rdata_o         : std_logic_vector (31 downto 0);
  signal csr_addr_i          : std_logic_vector (11 downto 0);

  -- CSR management unit internal signal
  signal csr_instr_req_replicated       : replicated_bit;
  signal csr_instr_done_replicated      : replicated_bit;
  signal csr_access_denied_o_replicated : replicated_bit;
  signal csr_rdata_o_replicated         : replicated_32b_reg;

  -- riscv 32x32bit register file 
  signal regfile   : regfile_replicated_array;
  signal WB_RD     : replicated_32b_reg;
  signal WB_RS2    : replicated_32b_reg;
  signal WB_RD_EN  : std_logic;
  signal WB_RS2_EN : std_logic;

  -- program counters --
  signal pc        : replicated_32b_reg;
  signal pc_WB     : std_logic_vector(31 downto 0);  -- pc_WB is pc entering stage WB
  signal pc_IE     : std_logic_vector(31 downto 0);  -- pc_IE is pc entering stage IE
  signal pc_ID     : std_logic_vector(31 downto 0);
  signal pc_ID_lat : std_logic_vector(31 downto 0);  -- pc_ID is PC entering ID stage
  signal pc_IF     : std_logic_vector(31 downto 0);  -- pc_IF is the actual pc

  -- instruction register and instr. propagation registers --
  signal instr_word_ID_lat      : std_logic_vector(31 downto 0);  -- latch needed for long-latency program memory
  signal instr_rvalid_ID        : std_logic;  -- validity bit at ID input
  signal instr_word_IE          : std_logic_vector(31 downto 0);
  signal instr_rvalid_IE        : std_logic;  -- validity bit at IE input
  signal instr_word_WB          : std_logic_vector(31 downto 0);
  signal instr_rvalid_WB        : std_logic;  -- idem
  signal decoded_instruction_IE : std_logic_vector(INSTR_SET_SIZE-1 downto 0);

  -- pc updater signals
  signal flush_count_ID                  : replicated_positive_integer;
  signal flush_thread                    : replicated_bit;
  signal flush_IF                        : replicated_bit;
  signal flush_ID                        : replicated_bit;
  signal flush_instruction_IF            : std_logic;
  signal flush_instruction_previous_IF   : std_logic;
  signal flush_instruction_ID            : std_logic;
  signal pc_update_enable                : replicated_bit;
  signal branch_condition_pending        : replicated_bit;
  signal except_condition_pending        : replicated_bit;
  signal mret_condition_pending          : replicated_bit;
  signal wfi_condition_pending           : replicated_bit;
  signal served_except_condition         : replicated_bit;
  signal served_mret_condition           : replicated_bit;
  signal served_irq                      : replicated_bit;
  signal set_branch_condition            : std_logic;
  signal set_except_condition            : std_logic;
  signal set_mret_condition              : std_logic;
  signal set_branch_condition_replicated : replicated_bit;
  signal set_wfi_condition_replicated    : replicated_bit;
  signal set_except_condition_replicated : replicated_bit;
  signal set_mret_condition_replicated   : replicated_bit;
  signal PC_offset                       : replicated_32b_reg;
  signal pc_except_value                 : replicated_32b_reg;
  signal taken_branch_pc_lat             : replicated_32b_reg;
  signal incremented_pc                  : replicated_32b_reg;
  signal mepc_incremented_pc             : replicated_32b_reg := (others => (others => '0'));
  signal mepc_interrupt_pc               : replicated_32b_reg := (others => (others => '0'));
  signal relative_to_PC                  : replicated_32b_reg;
  signal absolute_jump                   : std_logic          := '0';
  signal data_we_o_lat                   : std_logic;

  -- abs jump used to select the relative PC or absolute PC in jump/branch instruction
  -- only JALR does an absolute jump
  signal boot_pc : std_logic_vector(31 downto 0);

  --//parte probabilmente da eliminare
  -- signals for counting intructions
  signal clock_cycle         : std_logic_vector(63 downto 0);  -- RDCYCLE
  signal external_counter    : std_logic_vector(63 downto 0);  -- RDTIME
  signal instruction_counter : std_logic_vector(63 downto 0);  -- RDINSTRET

  --signal used by counters
  signal set_wfi_condition          : std_logic := '0';
  signal amo_load_skip              : std_logic;
  signal amo_instr                  : std_logic;
  signal amo_instr_lat              : std_logic;
  signal amoswap_writeback          : std_logic;
  signal amoswap_writeback_lat      : std_logic;
  signal swap_cycle                 : std_logic;
  signal swap_temp                  : std_logic_vector (31 downto 0);
  signal sw_mip                     : std_logic;
  signal harc_to_csr                : harc_range;
  signal jump_instr                 : std_logic;
  signal branch_instr               : std_logic;
  signal data_valid_waiting_counter : std_logic;

  -- auxiliary data memory interface signals
  signal data_addr_internal : std_logic_vector(31 downto 0);
  signal data_be_internal   : std_logic_vector(3 downto 0);

  --DeBug Unit signal and state
  type fsm_DBU_states is (RUNNING, HALT_REQ, HALT, SINGLE_STEP_REQ, SINGLE_STEP);
  signal state_DBU       : fsm_DBU_states;
  signal nextstate_DBU   : fsm_DBU_states;
  signal dbg_req_o       : std_logic;
  signal dbg_halted_o    : std_logic;
  signal dbg_ack_i       : std_logic;
  signal dbg_halt_req    : std_logic;
  signal dbg_resume_core : std_logic;
  signal dbg_ssh         : std_logic;
  signal dbg_sse         : std_logic;
  signal ebreak_instr    : std_logic;
  -- hardware context id at fetch, and propagated hardware context ids
  signal harc_IF         : harc_range;
  signal harc_IF_lat     : harc_range;
  signal harc_ID         : harc_range;
  signal harc_ID_lat     : harc_range;
  signal harc_IE         : harc_range;
  signal harc_WB         : harc_range;
  signal harc_IF_delayed : harc_range;  -- value one clock cycle before    
  signal harc_IE_delayed : harc_range;  -- value one clock cycle before  
  -- instruction operands
  signal S_Imm_IE        : std_logic_vector(11 downto 0);  -- unused
  signal I_Imm_IE        : std_logic_vector(11 downto 0);  -- unused
  signal SB_Imm_IE       : std_logic_vector(11 downto 0);  -- unused
  signal CSR_ADDR_IE     : std_logic_vector(11 downto 0);  -- unused
  signal RS1_Addr_IE     : std_logic_vector(4 downto 0);   -- unused
  signal RS2_Addr_IE     : std_logic_vector(4 downto 0);   -- unused
  signal RD_Addr_IE      : std_logic_vector(4 downto 0);   -- unused
  signal RS1_Data_IE     : std_logic_vector(31 downto 0);
  signal RS2_Data_IE     : std_logic_vector(31 downto 0);
  signal RD_Data_IE      : std_logic_vector(31 downto 0);  -- unused

  ---------------------------------------------------------------------------------------
  -- Subroutine implementing pc updating logic unit, to be replicated for max threads supported
  procedure pc_update(
    signal MTVEC                                           : in    std_logic_vector(31 downto 0);
    signal MCAUSE                                          : in    std_logic_vector(31 downto 0);
    signal instr_gnt_i, sw_mip, set_branch_condition       : in    std_logic;
    signal wfi_condition_pending                           : inout std_logic;
    signal set_wfi_condition                               : in    std_logic;
    signal branch_condition_pending                        : inout std_logic;
    signal irq_pending                                     : in    std_logic;
    signal set_except_condition                            : in    std_logic;
    signal except_condition_pending                        : inout std_logic;
    signal set_mret_condition                              : in    std_logic;
    signal mret_condition_pending                          : inout std_logic;
    signal pc                                              : out   std_logic_vector(31 downto 0);
    signal taken_branch_pc_lat                             : in    std_logic_vector(31 downto 0);
    signal incremented_pc                                  : in    std_logic_vector(31 downto 0);
    signal mepc_incremented_pc, mepc_interrupt_pc, boot_pc : in    std_logic_vector(31 downto 0);
    signal pc_update_enable                                : in    std_logic;
    signal served_except_condition                         : out   std_logic;
    signal served_mret_condition                           : out   std_logic;
    signal served_irq                                      : out   std_logic) is
  begin
    if pc_update_enable = '1' then

      -- interrupt service launched in the previous instr. cycle
      -- this is done for a second instr. cycle for properly synchronization of flushing
      if served_irq = '1' then
        pc                      <= MTVEC;
        served_except_condition <= '0';
        served_mret_condition   <= '0';
        served_irq              <= '0';

      -- nothing pending     
      elsif not (set_branch_condition = '1' or branch_condition_pending = '1')
        and not (irq_pending = '1')
        and not ((set_except_condition or except_condition_pending) = '1')
        and not ((set_mret_condition or mret_condition_pending) = '1')
        and not ((set_wfi_condition or wfi_condition_pending) = '1')
      then
        pc                      <= incremented_pc;
        served_except_condition <= '0';
        served_mret_condition   <= '0';
        served_irq              <= '0';

      -- branch pending 
      elsif set_branch_condition = '1' or branch_condition_pending = '1' then
        pc                       <= taken_branch_pc_lat;
        branch_condition_pending <= '0';
        served_except_condition  <= '0';
        served_mret_condition    <= '0';
        served_irq               <= '0';

      -- pending interrupt:
      elsif irq_pending = '1' then
        pc                      <= MTVEC;  -- standard riscv base trap vector                                     
        served_except_condition <= '0';
        served_mret_condition   <= '0';
        served_irq              <= '1';    -- commanding to update the CSRs
        wfi_condition_pending   <= '0';

      elsif set_wfi_condition = '1' or wfi_condition_pending = '1' then
        pc                      <= pc;
        wfi_condition_pending   <= '1';
        served_except_condition <= '0';
        served_mret_condition   <= '0';
        served_irq              <= '0';

      -- pending exception:
      elsif (set_except_condition or except_condition_pending) = '1' then
        pc                       <= MTVEC;  -- standard riscv base trap vector 
        except_condition_pending <= '0';
        served_except_condition  <= '1';    -- commanding to update the CSRs
        served_mret_condition    <= '0';
        served_irq               <= '0';

      -- pending mret from exception:          
      elsif (set_mret_condition or mret_condition_pending) = '1' and MCAUSE(31) = '0' then
        pc                      <= mepc_incremented_pc;
        mret_condition_pending  <= '0';
        served_except_condition <= '0';
        served_mret_condition   <= '1';  -- commanding to update the CSRs
        served_irq              <= '0';

      -- pending mret from interrupt:                       
      elsif (set_mret_condition or mret_condition_pending) = '1' and MCAUSE(31) = '1' then
        pc                      <= mepc_interrupt_pc;
        mret_condition_pending  <= '0';
        served_except_condition <= '0';
        served_mret_condition   <= '1';  -- commanding to update the CSRs
        served_irq              <= '0';

      else
        pc <= boot_pc;                  -- default, should never occur
      end if;
    -- end of pc value update ---                      
    else
      wfi_condition_pending    <= '1' when set_wfi_condition = '1' and sw_mip = '0';
      branch_condition_pending <= '1' when set_branch_condition = '1';
      except_condition_pending <= '1' when set_except_condition = '1';
      mret_condition_pending   <= '1' when set_mret_condition = '1';
      served_except_condition  <= '0';
      served_mret_condition    <= '0';
      served_irq               <= '0';
    end if;  -- insn gnt
  end pc_update;

--------------------------------------------------------------------------------------------------
----------------------- ARCHITECTURE BEGIN -------------------------------------------------------
begin

  -- check for microarchitecture configuration limit, up to 16 thread support.
  assert THREAD_POOL_SIZE < 2**THREAD_ID_SIZE
    report "threading configuration not supported"
    severity error;

  -- Memory fault signals
  load_err  <= data_gnt_i and data_err_i and not(data_we_o);
  store_err <= data_gnt_i and data_err_i and data_we_o;

  -- Memory address signal
  data_addr_o <= data_addr_internal(31 downto 2) & "00";
  data_be_o <= to_stdlogicvector(to_bitvector(data_be_internal) sll
                                 to_integer(unsigned(data_addr_internal(1 downto 0))));
  -- fixed connection of pc to output address port


  instr_addr_o <= pc_IF;


  debug_halted_o <= dbg_halted_o;
----------------------------------------------------------------------------------------------------
-- stage IF -- (instruction fetch)
----------------------------------------------------------------------------------------------------
-- This pipeline stage is implicitly present as the program memory is synchronous
-- with 1 cycle latency.
-- The fsm_IF manages the interface with program memory. 
-- The PC_IF is updated by a dedicated unit which is transparent to the fsm_IF.
----------------------------------------------------------------------------------------------------

  fsm_IF_nextstate : process(all)  -- acts as the control unit of the synchronous program memory
  begin
    if rst_ni = '0' then
      instr_req_o  <= '0';
      nextstate_IF <= normal;
    else
      case state_IF is
        when normal =>
          if busy_ID = '0' then
            instr_req_o <= '1';
            if instr_gnt_i = '1' then
              nextstate_IF <= normal;
            else
              nextstate_IF <= waiting;
            end if;
          else
            instr_req_o  <= '0';
            nextstate_IF <= normal;
          end if;
        when waiting =>
          if busy_ID = '0' then
            instr_req_o <= '1';
            if instr_gnt_i = '1' then
              nextstate_IF <= normal;
            else
              nextstate_IF <= waiting;
            end if;
          else
            instr_req_o  <= '0';
            nextstate_IF <= normal;
          end if;
          -- in the above code, in case of stall during a wait, the present access is aborted, 
          -- the instr_gnt will not come and the pc is not updated. Next access will repeat
          -- with same pc value. OK. This depends on how the memory works when req signal is aborted, 
          -- a more conservative implementation is needed if we want to be 100% sure 
          -- of correct operation

        when others =>                  -- should never occur
          nextstate_IF <= normal;
          instr_req_o  <= '0';
      end case;
    end if;
  end process;

  fsm_IF_register_state : process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      state_IF <= normal;
    elsif rising_edge(clk_i) then
      state_IF <= nextstate_IF;
    end if;
  end process;


  process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      pc_ID   <= (others => '0');
      harc_ID <= 0;
    elsif rising_edge(clk_i) then
      if instr_gnt_i = '1' then
        -- pc propagation
        pc_ID   <= pc_IF;
        -- harc propagation
        harc_ID <= harc_IF;
      end if;
    end if;
  end process;

  -- instr_rvalid_ID controller, needed to keep instr_valid_ID set during 
  -- stalls of the fetch stage. This is a synthesized mealy fsm
  process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      instr_rvalid_state <= '0';
    elsif rising_edge(clk_i) then
      instr_rvalid_state <= busy_ID and (instr_rvalid_i or instr_rvalid_state);
    end if;
  end process;
  instr_rvalid_ID <= (instr_rvalid_i or instr_rvalid_state);

  -- latch on program memory output, because memory output remains for 1 cycle only
  instr_word_ID_lat <= instr_rdata_i when instr_rvalid_i = '1';
  -- latches just to have pc, harc and ir updated together, probably useless
  pc_ID_lat         <= pc_ID         when instr_rvalid_ID = '1' else (others => '0') when rst_ni = '0';
  harc_ID_lat       <= harc_ID       when instr_rvalid_ID = '1' else 0 when rst_ni = '0';
--------------------------------------------------------------------- end of IF stage ---------------
-----------------------------------------------------------------------------------------------------






-----------------------------------------------------------------------------------------------------
-- Stage ID - (read operands)
-----------------------------------------------------------------------------------------------------
-- Actually does only source operand decoding and reading
-- This pipeline stage always takes one cycle latency
-----------------------------------------------------------------------------------------------------

  fsm_ID_sync : process(clk_i, rst_ni)  -- synch single state process
  begin
    if rst_ni = '0' then
      pc_IE           <= (others => '0');
      harc_IE         <= 0;
      instr_rvalid_IE <= '0';
    elsif rising_edge(clk_i) then
      if busy_IE = '1' then
        -- if amo_instr_lat = '1'then
        -- RS1_Data_IE     <= regfile(harc_IE)(rs1(instr_word_IE));
        -- RS2_Data_IE     <= regfile(harc_IE)(rs2(instr_word_IE));
        -- RD_Data_IE           <= regfile(harc_IE)(rd(instr_word_IE));
        -- end if;
        null;  -- do nothing and wait for the stall to finish; don't touch instr_rvalid_IE
      elsif instr_rvalid_ID = '0' then
        instr_rvalid_IE <= '0';         -- wait for a valid instruction

      elsif flush_instruction_ID = '1' then
        instr_rvalid_IE <= '0';         -- discard the valid instruction

      else                              -- propagate the instruction

        instr_rvalid_IE <= '1';
        instr_word_IE   <= instr_word_ID_lat;
        -- pc propagation
        pc_IE           <= pc_ID_lat;
        -- harc propagation
        harc_IE         <= harc_ID_lat;
        RS1_Addr_IE     <= std_logic_vector(to_unsigned(rs1(instr_word_ID_lat), 5));
        RS2_Addr_IE     <= std_logic_vector(to_unsigned(rs2(instr_word_ID_lat), 5));
        RD_Addr_IE      <= std_logic_vector(to_unsigned(rd(instr_word_ID_lat), 5));
        S_Imm_IE        <= std_logic_vector(to_unsigned(to_integer(unsigned(S_immediate(instr_word_ID_lat))), 12));
        I_Imm_IE        <= std_logic_vector(to_unsigned(to_integer(unsigned(I_immediate(instr_word_ID_lat))), 12));
        SB_Imm_IE       <= std_logic_vector(to_unsigned(to_integer(unsigned(SB_immediate(instr_word_ID_lat))), 12));
        CSR_ADDR_IE     <= std_logic_vector(to_unsigned(to_integer(unsigned(CSR_ADDR(instr_word_ID_lat))), 12));

        RS1_Data_IE <= regfile(harc_ID_lat)(rs1(instr_word_ID_lat)) when sw_mip = '0' else regfile(harc_IE)(rs1(instr_word_IE));
        RS2_Data_IE <= regfile(harc_ID_lat)(rs2(instr_word_ID_lat));
        RD_Data_IE  <= regfile(harc_ID)(rd(instr_word_ID_lat));
      end if;  -- instr. conditions
      if busy_IE /= '1' and instr_rvalid_ID /= '0' then
        -- process the instruction
        -- read data from the operand registers
        -- Decode Starts here
        amo_load_skip <= '0';
        case OPCODE(instr_word_ID_lat) is
          when OP_IMM =>
            if (FUNCT3(instr_word_ID_lat) = ADDI) then  -- ADDI instruction
              if (rd(instr_word_ID_lat) /= 0) then
                decoded_instruction_IE <= ADDI_pattern;
              else                                      --  R0 instruction
                decoded_instruction_IE <= NOP_pattern;
                null;
              end if;

            else
              if(rd(instr_word_ID_lat) /= 0) then
                case FUNCT3(instr_word_ID_lat) is
                  when SLTI =>          -- SLTI instruction
                    decoded_instruction_IE <= SLTI_pattern;
                  when SLTIU =>         -- SLTIU instruction
                    decoded_instruction_IE <= SLTIU_pattern;
                  when ANDI =>          -- ANDI instruction
                    decoded_instruction_IE <= ANDI_pattern;
                  when ORI =>           -- ORI instruction
                    decoded_instruction_IE <= ORI_pattern;
                  when XORI =>          -- XORI instruction
                    decoded_instruction_IE <= XORI_pattern;
                  when SLLI =>          -- SLLI instruction
                    decoded_instruction_IE <= SLLI_pattern;
                  when SRLI_SRAI =>
                    case FUNCT7(instr_word_ID_lat) is
                      when SRLI7 =>     -- SRLI instruction
                        decoded_instruction_IE <= SRLI7_pattern;
                      when SRAI7 =>     -- SRAI instruction
                        decoded_instruction_IE <= SRAI7_pattern;
                      when others =>  -- ILLEGAL_INSTRUCTION                                      
                        decoded_instruction_IE <= ILL_pattern;
                    end case;  -- FUNCT7(instr_word_ID_lat) cases
                  when others =>  -- ILLEGAL_INSTRUCTION                                  
                    decoded_instruction_IE <= ILL_pattern;
                end case;  -- FUNCT3(instr_word_ID_lat) cases   
              else              -- R0_INSTRUCTION                             
                decoded_instruction_IE <= NOP_pattern;
              end if;  -- if rd(instr_word_ID_lat) /=0
            end if;  -- ADDI if 
          when LUI =>                   -- LUI instruction
            if (rd(instr_word_ID_lat) /= 0) then
              decoded_instruction_IE <= LUI_pattern;
            else                        -- R0_INSTRUCTION
              decoded_instruction_IE <= NOP_pattern;
            end if;
          when AUIPC =>                 -- AUIPC instruction
            if (rd(instr_word_ID_lat) /= 0) then
              decoded_instruction_IE <= AUIPC_pattern;
            else                        -- R0_INSTRUCTION
              decoded_instruction_IE <= NOP_pattern;
            end if;
          when OP =>
            if (rd(instr_word_ID_lat) /= 0) then
              case FUNCT3(instr_word_ID_lat) is
                when ADD_SUB =>
                  case FUNCT7(instr_word_ID_lat) is
                    when ADD7 =>        --ADD instruction
                      decoded_instruction_IE <= ADD7_pattern;
                    when SUB7 =>        -- SUB instruction    
                      decoded_instruction_IE <= SUB7_pattern;
                    when others =>      -- ILLEGAL_INSTRUCTION
                      decoded_instruction_IE <= ILL_pattern;
                  end case;  -- FUNCT7(instr_word_ID_lat) cases                                   
                when SLT =>             -- SLT instruction 
                  decoded_instruction_IE <= SLT_pattern;
                when SLTU =>            -- SLTU instruction
                  decoded_instruction_IE <= SLTU_pattern;
                when ANDD =>            -- AND instruction
                  decoded_instruction_IE <= ANDD_pattern;
                when ORR =>             -- OR instruction
                  decoded_instruction_IE <= ORR_pattern;
                when XORR =>            -- XOR instruction        
                  decoded_instruction_IE <= XORR_pattern;
                when SLLL =>            -- SLL instruction        
                  decoded_instruction_IE <= SLLL_pattern;
                when SRLL_SRAA =>
                  case FUNCT7(instr_word_ID_lat) is
                    when SRLL7 =>       -- SRL instruction   
                      decoded_instruction_IE <= SRLL7_pattern;
                    when SRAA7 =>       -- SRA instruction
                      decoded_instruction_IE <= SRAA7_pattern;
                    when others =>  -- ILLEGAL_INSTRUCTION                                      
                      decoded_instruction_IE <= ILL_pattern;
                  end case;  -- FUNCT7(instr_word_ID_lat) cases
                when others =>  -- ILLEGAL_INSTRUCTION                                  
                  decoded_instruction_IE <= ILL_pattern;
              end case;  -- FUNCT3(instr_word_ID_lat) cases
            else                        -- R0_INSTRUCTION
              decoded_instruction_IE <= NOP_pattern;
            end if;

          when JAL =>                   -- JAL instruction
            decoded_instruction_IE <= JAL_pattern;

          when JALR =>                  -- JAL instruction
            decoded_instruction_IE <= JALR_pattern;

          when BRANCH =>      -- BRANCH instruction                         
            case FUNCT3(instr_word_ID_lat) is
              when BEQ =>               -- BEQ instruction   
                decoded_instruction_IE <= BEQ_pattern;
              when BNE =>               -- BNE instruction
                decoded_instruction_IE <= BNE_pattern;
              when BLT =>               -- BLT instruction   
                decoded_instruction_IE <= BLT_pattern;
              when BLTU =>              -- BLTU instruction
                decoded_instruction_IE <= BLTU_pattern;
              when BGE =>               -- BGE instruction
                decoded_instruction_IE <= BGE_pattern;
              when BGEU =>              -- BGEU instruction
                decoded_instruction_IE <= BGEU_pattern;
              when others =>  -- ILLEGAL_INSTRUCTION                      
                decoded_instruction_IE <= ILL_pattern;
            end case;  -- FUNCT3(instr_word_ID_lat) cases

          when LOAD =>                  -- LOAD instruction
            if (rd(instr_word_ID_lat) /= 0) then  -- is all in the next_state process
              case FUNCT3(instr_word_ID_lat) is
                when LW =>
                  decoded_instruction_IE <= LW_pattern;
                when LH =>
                  decoded_instruction_IE <= LH_pattern;
                when LHU =>
                  decoded_instruction_IE <= LHU_pattern;
                when LB =>
                  decoded_instruction_IE <= LB_pattern;
                when LBU =>
                  decoded_instruction_IE <= LBU_pattern;
                when others =>          -- ILLEGAL_INSTRUCTION
                  decoded_instruction_IE <= ILL_pattern;
              end case;
            else                        -- R0_INSTRUCTION
              decoded_instruction_IE <= NOP_pattern;
            end if;

          when STORE =>                 -- STORE instruction
            case FUNCT3(instr_word_ID_lat) is
              when SW =>                -- is all in the next_state process
                decoded_instruction_IE <= SW_pattern;
              when SH =>
                decoded_instruction_IE <= SH_pattern;
              when SB =>
                decoded_instruction_IE <= SB_pattern;
              when others =>  -- ILLEGAL_INSTRUCTION                              
                decoded_instruction_IE <= ILL_pattern;
            end case;

          when MISC_MEM =>
            case FUNCT3(instr_word_ID_lat) is
              when FENCE =>             -- FENCE instruction
                decoded_instruction_IE <= FENCE_pattern;
              when FENCEI =>            -- FENCEI instruction
                decoded_instruction_IE <= FENCEI_pattern;
              when others =>            -- ILLEGAL_INSTRUCTION
                decoded_instruction_IE <= ILL_pattern;
            end case;  -- FUNCT3(instr_word_ID_lat) cases

          when SYSTEM =>
            case FUNCT3(instr_word_ID_lat) is
              when PRIV =>
                if (rs1(instr_word_ID_lat) = 0 and rd(instr_word_ID_lat) = 0) then
                  case FUNCT12(instr_word_ID_lat) is
                    when ECALL =>       -- ECALL instruction
                      decoded_instruction_IE <= ECALL_pattern;
                    when EBREAK =>      -- EBREAK instruction       
                      decoded_instruction_IE <= EBREAK_pattern;
                    when mret =>        -- mret instruction   
                      decoded_instruction_IE <= MRET_pattern;
                    when WFI =>         -- WFI instruction     
                      decoded_instruction_IE <= WFI_pattern;
                    when others =>  -- ILLEGAL_INSTRUCTION                                              
                      decoded_instruction_IE <= ILL_pattern;
                  end case;  -- FUNCT12(instr_word_ID_lat) cases
                else  -- ILLEGAL_INSTRUCTION                            
                  decoded_instruction_IE <= ILL_pattern;
                end if;
              when CSRRW =>
                decoded_instruction_IE <= CSRRW_pattern;
              when CSRRS =>
                if(rd(instr_word_ID_lat) /= 0) then
                  decoded_instruction_IE <= CSRRS_pattern;
                else                    -- R0_INSTRUCTION
                  decoded_instruction_IE <= NOP_pattern;
                end if;
              when CSRRC =>
                if(rd(instr_word_ID_lat) /= 0) then
                  decoded_instruction_IE <= CSRRC_pattern;
                else                    -- R0_INSTRUCTION
                  decoded_instruction_IE <= NOP_pattern;
                end if;
              when CSRRWI =>
                decoded_instruction_IE <= CSRRWI_pattern;
              when CSRRSI =>
                if(rd(instr_word_ID_lat) /= 0) then
                  decoded_instruction_IE <= CSRRSI_pattern;
                else                    -- R0_INSTRUCTION
                  decoded_instruction_IE <= NOP_pattern;
                end if;
              when CSRRCI =>
                if(rd(instr_word_ID_lat) /= 0) then
                  decoded_instruction_IE <= CSRRCI_pattern;
                else                    -- R0_INSTRUCTION
                  decoded_instruction_IE <= NOP_pattern;
                end if;
              when others =>  -- ILLEGAL_INSTRUCTION                      
                decoded_instruction_IE <= ILL_pattern;
            end case;  -- FUNCT3(instr_word_ID_lat) cases

          when AMO =>
            case FUNCT3(instr_word_ID_lat) is
              when SINGLE =>
                decoded_instruction_IE <= AMOSWAP_pattern;
                if(rd(instr_word_ID_lat) /= 0) then
                  amo_load_skip <= '0';
                elsif (rd(instr_word_ID_lat) = 0) then
                  amo_load_skip <= '1';
                end if;
              when others =>            -- ILLEGAL_INSTRUCTION
                decoded_instruction_IE <= ILL_pattern;
            end case;
          when others =>                -- ILLEGAL_INSTRUCTION          
            decoded_instruction_IE <= ILL_pattern;
        end case;  -- OPCODE(instr_word_ID_lat) cases                           
        -- Decode OF INSTRUCTION (END) --------------------------

      end if;  -- instr. conditions
    end if;  -- clk
  end process;

  fsm_ID_comb : process(all)
  begin
    if busy_IE = '1' then
      busy_ID <= '1';  -- wait for the stall to finish, block new instructions 
    elsif instr_rvalid_ID = '0' then
      busy_ID <= '0';                   -- wait for a valid instruction        
    elsif flush_instruction_ID = '1' then
      busy_ID <= '0';  -- discard the valid instruction, request new one       
    else
      busy_ID <= '0';                   -- process the instruction
    end if;  -- instr. conditions
  end process;
---------------------------------------------------------------------- end of ID stage -------------
----------------------------------------------------------------------------------------------------





----------------------------------------------------------------------------------------------------
-- stage IE -- (instruction decode/Execute)
----------------------------------------------------------------------------------------------------
-- This stage is composed of an fsm unit fsm_IE that performs synchronous operations,
-- and drives the control signals for accessing data memory and stalling the pipeline if needed
-- fsm_IE may invoke separate units for handling specific instructions (exceptions, csrs)
----------------------------------------------------------------------------------------------------

  fsm_IE_sync : process(clk_i, rst_ni)

    -- pragma translate_off
    variable row : line;  -- local variable for instruction tracing, not synthesizable
    -- pragma translate_on

  begin
    if rst_ni = '0' then
      for h in harc_range loop
        WB_RD(h) <= std_logic_vector(to_unsigned(0, 32));
      end loop;
      WB_RD_EN            <= '0';
      instruction_counter <= std_logic_vector(to_unsigned(0, 64));
      csr_instr_req       <= '0';
      csr_op_i            <= (others => '0');
      csr_wdata_i         <= (others => '0');
      csr_addr_i         <= (others => '0');
    elsif rising_edge(clk_i) then
      case state_IE is                  -- stage state
        when sleep =>
          null;
        when reset =>
          null;
        when first_boot =>
          null;
        when debug =>
          null;
        when normal =>
          if instr_rvalid_IE = '0' then
            instr_rvalid_WB <= '0';
          -- in the 4 stage version we will have conditions on flush_thread_IE and busy_WB 
          -- in all states of the IE stage, and similarly in the comb process, just
          -- like we did in the ID stage.
          else                          -- process the instruction
            instruction_counter <= std_logic_vector(unsigned(instruction_counter)+1);
            pc_WB               <= pc_IE;
            instr_rvalid_WB     <= '1';
            instr_word_WB       <= instr_word_IE;
            harc_WB             <= harc_IE;
            WB_RS2_EN           <= '0';
            -- pragma translate_off
            --write(row, OPCODE(instr_word_IE)); -- Writes OPCODE(instr_word_IE) value to line GGG
            hwrite(row, pc_IE);  --GGG I write on file instruction machine code and its position on memory program (pc_IE)
            write(row, '_');            --GGG
            hwrite(row, instr_word_IE);    --GGG
            writeline(file_handler, row);  -- Writes line to instr. trace file
            -- pragma translate_on
            -- EXECUTE OF INSTRUCTION -------------------------------------------


            if decoded_instruction_IE(ADDI_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <= std_logic_vector(signed(RS1_Data_IE)+
                                                 signed(I_immediate(instr_word_IE)));
            end if;

            if decoded_instruction_IE(SLTI_bit_position) = '1' then
              if (signed(RS1_Data_IE) < signed (I_immediate(instr_word_IE))) then
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(1, 32));
              else
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(0, 32));
              end if;
            end if;

            if decoded_instruction_IE(SLTIU_bit_position) = '1' then
              if (unsigned(RS1_Data_IE) < unsigned (I_immediate(instr_word_IE))) then
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(1, 32));
              else
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(0, 32));
              end if;
            end if;


            if decoded_instruction_IE(ANDI_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE and I_immediate(instr_word_IE);
            end if;

            if decoded_instruction_IE(ORI_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE or I_immediate(instr_word_IE);
            end if;

            if decoded_instruction_IE(XORI_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE xor I_immediate(instr_word_IE);
            end if;

            if decoded_instruction_IE(SLLI_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  sll to_integer(unsigned(SHAMT(instr_word_IE))));
            end if;

            if decoded_instruction_IE(SRLI7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  srl to_integer(unsigned(SHAMT(instr_word_IE))));
            end if;

            if decoded_instruction_IE(SRAI7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  sra to_integer(unsigned(SHAMT(instr_word_IE))));
            end if;

            if decoded_instruction_IE(LUI_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= U_immediate(instr_word_IE);
            end if;

            if decoded_instruction_IE(AUIPC_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <= std_logic_vector(signed(U_immediate(instr_word_IE))
                                                 + signed(pc_IE));  --GGG before it was signed(pc)
            end if;

            if decoded_instruction_IE(ADD7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <= std_logic_vector(signed(RS1_Data_IE)
                                                 + signed(RS2_Data_IE));
            end if;

            if decoded_instruction_IE(SUB7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <= std_logic_vector(signed(RS1_Data_IE)
                                                 - signed(RS2_Data_IE));
            end if;

            if decoded_instruction_IE(SLT_bit_position) = '1' then
              WB_RD_EN <= '1';
              if (signed(RS1_Data_IE) < signed (RS2_Data_IE)) then
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(1, 32));
              else
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(0, 32));
              end if;
            end if;

            if decoded_instruction_IE(SLTU_bit_position) = '1' then
              WB_RD_EN <= '1';
              if (unsigned(RS1_Data_IE) < unsigned (RS2_Data_IE)) then
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(1, 32));
              else
                WB_RD(harc_IE) <= std_logic_vector(to_unsigned(0, 32));
              end if;
            end if;

            if decoded_instruction_IE(ANDD_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE and RS2_Data_IE;
            end if;

            if decoded_instruction_IE(ORR_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE or RS2_Data_IE;
            end if;

            if decoded_instruction_IE(XORR_bit_position) = '1' then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= RS1_Data_IE xor RS2_Data_IE;
            end if;

            if decoded_instruction_IE(SLLL_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  sll to_integer(unsigned(RS2_Data_IE
                                                          (4 downto 0))));
            end if;
            if decoded_instruction_IE(SRLL7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  srl to_integer(unsigned(RS2_Data_IE
                                                          (4 downto 0))));
            end if;

            if decoded_instruction_IE(SRAA7_bit_position) = '1' then
              WB_RD_EN <= '1';
              WB_RD(harc_IE) <=
                to_stdlogicvector(to_bitvector(RS1_Data_IE)
                                  sra to_integer(unsigned(RS2_Data_IE
                                                          (4 downto 0))));
            end if;

            if decoded_instruction_IE(JAL_bit_position) = '1' or decoded_instruction_IE(JALR_bit_position) = '1' then
              if (rd(instr_word_IE) /= 0) then
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= std_logic_vector(unsigned(pc_IE) + "100");
              else                      -- plain unconditional jump
                WB_RD_EN <= '0';
                null;
              end if;
            end if;



            if decoded_instruction_IE(BEQ_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(BNE_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(BLT_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(BLTU_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(BGE_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(BGEU_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(LW_bit_position) = '1' then
              WB_RD_EN <= '0';
              if(data_addr_internal(1 downto 0) = "00") then
                if (load_err = '1') then
                  WB_RD_EN                 <= '0';
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                elsif (store_err = '1') then
                  WB_RD_EN                 <= '0';
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                end if;
              else
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= LOAD_MISALIGNED_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(LH_bit_position) = '1' or decoded_instruction_IE(LHU_bit_position) = '1' then
              WB_RD_EN <= '0';
              if(data_addr_internal(0) = '0') then
                if (load_err = '1') then
                  WB_RD_EN                 <= '0';
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                elsif (store_err = '1') then
                  WB_RD_EN                 <= '0';
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                end if;
              else
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= LOAD_MISALIGNED_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(LB_bit_position) = '1' or decoded_instruction_IE(LBU_bit_position) = '1' then
              WB_RD_EN <= '0';
              if (load_err = '1') then
                WB_RD_EN                 <= '0';
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
              elsif (store_err = '1') then
                WB_RD_EN                 <= '0';
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(SW_bit_position) = '1' then
              WB_RD_EN <= '0';

              if data_addr_internal = x"0000FF00" then
                csr_op_i      <= CSRRW;
                csr_instr_req <= '1';
                csr_wdata_i   <= RS1_Data_IE;
                csr_addr_i     <= MIP_ADDR;
                harc_to_csr <= 0;
              elsif data_addr_internal = x"0000FF04" then
                csr_op_i      <= CSRRW;
                csr_instr_req <= '1';
                csr_wdata_i   <= RS1_Data_IE;
                csr_addr_i     <= MIP_ADDR;
                harc_to_csr <= 1;
              elsif data_addr_internal = x"0000FF08" then
                csr_op_i      <= CSRRW;
                csr_instr_req <= '1';
                csr_wdata_i   <= RS1_Data_IE;
                csr_addr_i     <= MIP_ADDR;
                harc_to_csr <= 2;
              elsif data_addr_internal = x"0000FF0C" then
                csr_op_i      <= CSRRW;
                csr_instr_req <= '1';
                csr_wdata_i   <= RS1_Data_IE;
                csr_addr_i     <= MIP_ADDR;
                harc_to_csr <= 3;
              end if;

              if(data_addr_internal(1 downto 0) = "00") then
                if (load_err = '1') then
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                elsif (store_err = '1') then
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                end if;
              else
                                        --misaligned_address <= data_addr_internal;
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= STORE_MISALIGNED_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(SH_bit_position) = '1' then
              WB_RD_EN <= '0';
              if(data_addr_internal(0) = '0') then
                if (load_err = '1') then
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                elsif (store_err = '1') then
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                end if;
              else
                                        --misaligned_address <= data_addr_internal;
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= STORE_MISALIGNED_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(SB_bit_position) = '1' then
              WB_RD_EN <= '0';
              if (load_err = '1') then
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
              elsif (store_err = '1') then
                pc_except_value(harc_IE) <= pc_IE;
                csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
              end if;
            end if;

            if decoded_instruction_IE(FENCE_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;                     -- WARNING: still not implemented
            end if;

            if decoded_instruction_IE(FENCEI_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;                     -- WARNING: still not implemented
            end if;

            if decoded_instruction_IE(ECALL_bit_position) = '1' then
              WB_RD_EN                 <= '0';
              csr_wdata_i              <= ECALL_EXCEPT_CODE;
              pc_except_value(harc_IE) <= pc_IE;
            end if;

            if decoded_instruction_IE(EBREAK_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;                     -- WARNING: still not implemented
            end if;

            if decoded_instruction_IE(MRET_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;                     -- managed by combinat. signal
            end if;

            if decoded_instruction_IE(WFI_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(ERET_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

            if decoded_instruction_IE(CSRRW_bit_position) = '1' then
              WB_RD_EN      <= '0';
              csr_op_i      <= FUNCT3(instr_word_IE);
              csr_instr_req <= '1';
              csr_wdata_i   <= RS1_Data_IE;
              csr_addr_i     <= std_logic_vector(to_unsigned(to_integer(unsigned(CSR_ADDR(instr_word_IE))), 12));
              harc_to_csr <= harc_IE;
            end if;

            if decoded_instruction_IE(CSRRC_bit_position) = '1' or decoded_instruction_IE(CSRRS_bit_position) = '1' then
              WB_RD_EN      <= '0';
              csr_op_i      <= FUNCT3(instr_word_IE);
              csr_instr_req <= '1';
              csr_wdata_i   <= RS1_Data_IE;
              csr_addr_i     <= std_logic_vector(to_unsigned(to_integer(unsigned(CSR_ADDR(instr_word_IE))), 12));
              harc_to_csr <= harc_IE;
            end if;

            if decoded_instruction_IE(CSRRWI_bit_position) = '1' then
              WB_RD_EN      <= '0';
              csr_op_i      <= FUNCT3(instr_word_IE);
              csr_instr_req <= '1';
              csr_wdata_i   <= std_logic_vector(resize(to_unsigned(rs1(instr_word_IE), 5), 32));
              csr_addr_i     <= std_logic_vector(to_unsigned(to_integer(unsigned(CSR_ADDR(instr_word_IE))), 12));
              harc_to_csr <= harc_IE;
            end if;

            if decoded_instruction_IE(CSRRSI_bit_position) = '1'or decoded_instruction_IE(CSRRCI_bit_position) = '1' then
              WB_RD_EN      <= '0';
              csr_op_i      <= FUNCT3(instr_word_IE);
              csr_instr_req <= '1';
              csr_wdata_i   <= std_logic_vector(resize(to_unsigned(rs1(instr_word_IE), 5), 32));
              csr_addr_i     <= std_logic_vector(to_unsigned(to_integer(unsigned(CSR_ADDR(instr_word_IE))), 12));
              harc_to_csr <= harc_IE;
            end if;

            if decoded_instruction_IE(AMOSWAP_bit_position) = '1' then
              if amoswap_writeback_lat = '0' and amo_load_skip = '0' then
                if(data_addr_internal(1 downto 0) = "00") then
                  swap_cycle <= '0';
                  WB_RD_EN   <= '0';
                  if (load_err = '1') then
                    WB_RD_EN                 <= '0';
                    pc_except_value(harc_IE) <= pc_IE;
                    csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                  elsif (store_err = '1') then
                    WB_RD_EN                 <= '0';
                    pc_except_value(harc_IE) <= pc_IE;
                    csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                  end if;
                else
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= LOAD_MISALIGNED_EXCEPT_CODE;
                end if;

              elsif amoswap_writeback_lat = '1' or amo_load_skip = '1' then
                WB_RD_EN <= '0';
                if(data_addr_internal(1 downto 0) = "00") then
                  if (load_err = '1') then
                    pc_except_value(harc_IE) <= pc_IE;
                    csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
                  elsif (store_err = '1') then
                    pc_except_value(harc_IE) <= pc_IE;
                    csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
                  end if;
                else
                                        --misaligned_address <= data_addr_internal;
                  pc_except_value(harc_IE) <= pc_IE;
                  csr_wdata_i              <= STORE_MISALIGNED_EXCEPT_CODE;
                end if;
              end if;
            end if;


            if decoded_instruction_IE(ILL_bit_position) = '1' then
              WB_RD_EN                 <= '0';
              csr_wdata_i              <= ILLEGAL_INSN_EXCEPT_CODE;
              pc_except_value(harc_IE) <= pc_IE;
            end if;

            if decoded_instruction_IE(NOP_bit_position) = '1' then
              WB_RD_EN <= '0';
              null;
            end if;

          -- EXECUTE OF INSTRUCTION (END) --------------------------
          end if;  -- instr_rvalid_IE values

        when data_valid_waiting =>
          if instr_rvalid_IE = '0' then
            null;
          else                          -- process the instruction          
            if (load_err = '1') then
              WB_RD_EN                 <= '0';
              pc_except_value(harc_IE) <= pc_IE;
              csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
            elsif (store_err = '1') then
              WB_RD_EN                 <= '0';
              pc_except_value(harc_IE) <= pc_IE;
              csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
            elsif (data_rvalid_i = '1' and OPCODE(instr_word_IE) = LOAD and rd(instr_word_IE) /= 0) then
              if decoded_instruction_IE(LW_bit_position) = '1' then
                if(data_addr_internal(1 downto 0) = "00") then  -- LW instruction
                  WB_RD_EN       <= '1';
                  WB_RD(harc_IE) <= data_rdata_i;
                end if;
              end if;

              if decoded_instruction_IE(LH_bit_position) = '1' then  -- LH instruction
                case data_addr_internal(1 downto 0) is
                  when "00" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(15 downto 0)), 32));
                  when "01" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(23 downto 8)), 32));
                  when "10" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(31 downto 16)), 32));
                  when others =>
                    null;               --GGG maybe an exception...
                end case;
              end if;

              if decoded_instruction_IE(LHU_bit_position) = '1' then  -- LHU instruction
                case data_addr_internal(1 downto 0) is
                  when "00" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(15 downto 0)), 32));
                  when "01" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(23 downto 8)), 32));
                  when "10" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(31 downto 16)), 32));
                  when others =>
                    null;               --GGG maybe an exception...
                end case;
              end if;

              if decoded_instruction_IE(LB_bit_position) = '1' then  -- LB instruction
                case data_addr_internal(1 downto 0) is
                  when "00" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(7 downto 0)), 32));
                  when "01" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(15 downto 8)), 32));
                  when "10" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(23 downto 16)), 32));
                  when "11" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(signed(data_rdata_i(31 downto 24)), 32));
                  when others =>
                    null;               --GGG maybe it's an exception...
                end case;
              end if;
              if decoded_instruction_IE(LBU_bit_position) = '1' then  -- LBU instruction
                case data_addr_internal(1 downto 0) is
                  when "00" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(7 downto 0)), 32));
                  when "01" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(15 downto 8)), 32));
                  when "10" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(23 downto 16)), 32));
                  when "11" =>
                    WB_RD_EN <= '1';
                    WB_RD(harc_IE) <=
                      std_logic_vector(resize(unsigned(data_rdata_i(31 downto 24)), 32));
                  when others =>
                    WB_RD_EN <= '0';
                    null;               --GGG maybe it's an exception...
                end case;
              end if;
            elsif (data_rvalid_i = '1' and amo_instr_lat = '1' and rd(instr_word_IE) /= 0) then
              if amoswap_writeback_lat = '0' then
                WB_RD_EN       <= '1';
                WB_RD(harc_IE) <= data_rdata_i;
              end if;
            else
              WB_RD_EN <= '0';
              null;
            end if;
          end if;  -- instr_rvalid_IE
        when data_grant_waiting =>      -- wait on data_grant_waiting signal
          if instr_rvalid_IE = '0' then

            else                        -- process the instruction
            if (load_err = '1') then
              WB_RD_EN                 <= '0';
              pc_except_value(harc_IE) <= pc_IE;
              csr_wdata_i              <= LOAD_ERROR_EXCEPT_CODE;
            elsif (store_err = '1') then
              WB_RD_EN                 <= '0';
              pc_except_value(harc_IE) <= pc_IE;
              csr_wdata_i              <= STORE_ERROR_EXCEPT_CODE;
            end if;
          end if;  -- instr_rvalid 
        when csr_instr_wait_state =>
          csr_instr_req <= '0';
          if (csr_instr_done = '1' and csr_access_denied_o = '0') then
            if (rd(instr_word_IE) /= 0) then
              WB_RD_EN       <= '1';
              WB_RD(harc_IE) <= csr_rdata_o;
            else
              WB_RD_EN <= '0';
              null;
            end if;
          elsif (csr_instr_done = '1' and csr_access_denied_o = '1') then  -- ILLEGAL_INSTRUCTION
            WB_RD_EN                 <= '0';
            csr_wdata_i              <= ILLEGAL_INSN_EXCEPT_CODE;
            pc_except_value(harc_IE) <= pc_IE;
          else
            WB_RD_EN <= '0';
          end if;

      end case;  -- fsm_IE state cases
    end if;  -- reset, clk_i
  end process;

  fsm_IE_comb : process(all)

    variable PC_offset_wires                  : replicated_32b_reg;
    variable data_addr_internal_wires         : std_logic_vector (31 downto 0);
    variable data_wdata_o_wires               : std_logic_vector (31 downto 0);
    variable data_be_internal_wires           : std_logic_vector (3 downto 0);
    variable data_we_o_wires                  : std_logic;
    variable absolute_jump_wires              : std_logic;
    variable busy_IE_wires                    : std_logic;
    variable set_except_condition_wires       : std_logic;
    variable set_branch_condition_wires       : std_logic;
    variable set_mret_condition_wires         : std_logic;
    variable jump_instr_wires                 : std_logic;
    variable branch_instr_wires               : std_logic;
    variable ebreak_instr_wires               : std_logic;
    variable amo_instr_wires                  : std_logic;
    variable dbg_ack_i_wires                  : std_logic;
    variable data_valid_waiting_counter_wires : std_logic;
    variable data_req_o_wires                 : std_logic;
    variable amoswap_writeback_wires          : std_logic;
    variable nextstate_IE_wires               : fsm_IE_states;

  begin

    PC_offset_wires                  := (others => (others => '0'));
    data_addr_internal_wires         := "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
    data_wdata_o_wires               := "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
    data_be_internal_wires           := "ZZZZ";
    data_we_o_wires                  := 'Z';
    absolute_jump_wires              := '0';
    busy_IE_wires                    := '0';
    set_except_condition_wires       := '0';
    set_branch_condition_wires       := '0';
    set_mret_condition_wires         := '0';
    jump_instr_wires                 := '0';
    branch_instr_wires               := '0';
    ebreak_instr_wires               := '0';
    amo_instr_wires                  := '0';
    dbg_ack_i_wires                  := '0';
    data_valid_waiting_counter_wires := '0';
    data_req_o_wires                 := '0';
    amoswap_writeback_wires          := '0';
    nextstate_IE_wires               := sleep;
    set_wfi_condition                <= '0';
    sw_mip                           <= '0';
    if rst_ni = '0' then
      if fetch_enable_i = '1' then
        null;
      else
        busy_IE_wires := '1';
      end if;
      nextstate_IE_wires := normal;  -- ignored, but allows combinational synthesis
    else
      case state_IE is                  -- stage status
        when sleep =>
          if dbg_req_o = '1' then
            dbg_ack_i_wires    := '1';
            busy_IE_wires      := '1';
            nextstate_IE_wires := sleep;
          elsif irq_pending(harc_IE) = '1' or fetch_enable_i = '1' then
            nextstate_IE_wires := normal;
          else
            busy_IE_wires      := '1';
            nextstate_IE_wires := sleep;
          end if;

        when reset =>
          if dbg_req_o = '1' then
            dbg_ack_i_wires    := '1';
            busy_IE_wires      := '1';
            nextstate_IE_wires := reset;
          elsif fetch_enable_i = '0' then
            nextstate_IE_wires := reset;
            busy_IE_wires      := '1';
          else
            nextstate_IE_wires := normal;
          end if;

        when first_boot =>
          nextstate_IE_wires := normal;

        when debug =>
          dbg_ack_i_wires := '1';
          if dbg_req_o = '0' then
            nextstate_IE_wires := normal;
          else
            nextstate_IE_wires := debug;
            busy_IE_wires      := '1';
          end if;

        when normal =>
          -- in the 4 stage version we will have conditions on flush_thread_IE and busy_WB 
          -- in all states of the IE stage, and similarly in the comb process, just
          -- like we did in the ID stage.
          if instr_rvalid_IE = '0' then
            nextstate_IE_wires := normal;
          else                          -- process the instruction
            -- EXECUTE OF INSTRUCTION ---------------------

            if decoded_instruction_IE(ADDI_bit_position) = '1' or decoded_instruction_IE(SLTI_bit_position) = '1' 
               or decoded_instruction_IE(SLTIU_bit_position) = '1' or decoded_instruction_IE(ANDI_bit_position) = '1' 
               or decoded_instruction_IE(ORI_bit_position) = '1' or decoded_instruction_IE(XORI_bit_position) = '1' 
               or decoded_instruction_IE(SLLI_bit_position) = '1' or decoded_instruction_IE(SRLI7_bit_position) = '1' 
               or decoded_instruction_IE(SRAI7_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(LUI_bit_position) = '1' or decoded_instruction_IE(AUIPC_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(ADD7_bit_position) = '1' or decoded_instruction_IE(SUB7_bit_position) = '1'
              or decoded_instruction_IE(SLT_bit_position) = '1' or decoded_instruction_IE(SLTU_bit_position) = '1'
              or decoded_instruction_IE(ANDD_bit_position) = '1' or decoded_instruction_IE(ORR_bit_position) = '1'
              or decoded_instruction_IE(XORR_bit_position) = '1' or decoded_instruction_IE(SLLL_bit_position) = '1'
              or decoded_instruction_IE(SRLL7_bit_position) = '1' or decoded_instruction_IE(SRAA7_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(FENCE_bit_position) = '1' or decoded_instruction_IE(FENCEI_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(JAL_bit_position) = '1' then  -- JAL instruction
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              jump_instr_wires           := '1';
              set_branch_condition_wires := '1';
              PC_offset_wires(harc_IE)   := UJ_immediate(instr_word_IE);
            end if;

            if decoded_instruction_IE(JALR_bit_position) = '1' then  --JALR instruction
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              set_branch_condition_wires := '1';
              PC_offset_wires(harc_IE) := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(I_immediate(instr_word_IE)))
                                          and X"FFFFFFFE";  -- bitwise and to set '0' the LSB
              jump_instr_wires    := '1';
              absolute_jump_wires := '1';
            end if;

            if decoded_instruction_IE(BEQ_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (unsigned(RS1_Data_IE) = unsigned(RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(BNE_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (unsigned(RS1_Data_IE) /= unsigned(RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(BLT_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (signed(RS1_Data_IE) < signed (RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(BLTU_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (unsigned(RS1_Data_IE) < unsigned (RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(BGE_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (signed(RS1_Data_IE) >= signed (RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(BGEU_bit_position) = '1' then
              if dbg_req_o = '1' then
                nextstate_IE_wires := debug;
                dbg_ack_i_wires    := '1';
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := normal;
              end if;
              branch_instr_wires       := '1';
              PC_offset_wires(harc_IE) := SB_immediate(instr_word_IE);
              if (unsigned(RS1_Data_IE) >= unsigned (RS2_Data_IE)) then
                set_branch_condition_wires := '1';
              end if;
            end if;

            if decoded_instruction_IE(LW_bit_position) = '1' then  -- LW instruction
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE) + signed(I_immediate(instr_word_IE)));
              data_be_internal_wires   := "1111";
              data_req_o_wires         := '1';
              data_we_o_wires          := '0';
              if(data_addr_internal_wires(1 downto 0) = "00") then
                if (load_err = '1') then
                  nextstate_IE_wires         := normal;
                  set_except_condition_wires := '1';
                elsif (store_err = '1') then
                  nextstate_IE_wires         := normal;
                  set_except_condition_wires := '1';
                elsif data_gnt_i = '1' then
                  nextstate_IE_wires := data_valid_waiting;
                  busy_IE_wires      := '1';
                else
                  nextstate_IE_wires := data_grant_waiting;
                  busy_IE_wires      := '1';
                end if;
              else
                set_except_condition_wires := '1';
                busy_IE_wires              := '1';
              end if;
            end if;

            if decoded_instruction_IE(LH_bit_position) = '1' or decoded_instruction_IE(LHU_bit_position) = '1' then  -- LH instruction -- LHU instruction
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE) + signed(I_immediate(instr_word_IE)));
              data_req_o_wires         := '1';
              data_we_o_wires          := '0';
              data_be_internal_wires   := "0011";
              if(data_addr_internal_wires(0) = '0') then
                if (load_err = '1') then
                  nextstate_IE_wires         := normal;
                  set_except_condition_wires := '1';
                elsif (store_err = '1') then
                  nextstate_IE_wires         := normal;  -- was except_wait_state;
                  set_except_condition_wires := '1';
                elsif data_gnt_i = '1' then
                  nextstate_IE_wires := data_valid_waiting;
                  busy_IE_wires      := '1';
                else
                  nextstate_IE_wires := data_grant_waiting;
                  busy_IE_wires      := '1';
                end if;
              else
                set_except_condition_wires := '1';
                busy_IE_wires              := '1';
              end if;
            end if;

            if decoded_instruction_IE(LB_bit_position) = '1' or decoded_instruction_IE(LBU_bit_position) = '1' then  -- LB instruction -- LBU instruction
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE) + signed(I_immediate(instr_word_IE)));
              data_req_o_wires         := '1';
              data_we_o_wires          := '0';
              data_be_internal_wires   := "0001";
              if (load_err = '1') then
                nextstate_IE_wires         := normal;  -- was except_wait_state;
                set_except_condition_wires := '1';
              elsif (store_err = '1') then
                nextstate_IE_wires         := normal;  -- was except_wait_state;
                set_except_condition_wires := '1';
              elsif data_gnt_i = '1' then
                nextstate_IE_wires := data_valid_waiting;
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := data_grant_waiting;
                busy_IE_wires      := '1';
              end if;
            end if;

            if decoded_instruction_IE(SW_bit_position) = '1' then

              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(S_immediate(instr_word_IE)));
              --data_wdata_o_wires         <= RS2_Data_IE;  GG it is not so easy

              if data_addr_internal = x"0000FF00" or data_addr_internal = x"0000FF04" 
                 or data_addr_internal = x"0000FF08" or data_addr_internal = x"0000FF0C" then -- act as a csr!!
                busy_IE_wires      := '1';
                nextstate_IE_wires := csr_instr_wait_state;
              else
                data_we_o_wires := '1';                    -- is a writing
                case data_addr_internal_wires(1 downto 0) is
                  when "00" =>
                    data_wdata_o_wires := RS2_Data_IE(31 downto 0);
                  when "01" =>
                    data_wdata_o_wires := RS2_Data_IE(23 downto 0) & std_logic_vector(to_unsigned(0, 8));
                  when "10" =>
                    data_wdata_o_wires := RS2_Data_IE(15 downto 0) & std_logic_vector(to_unsigned(0, 16));
                  when "11" =>
                    data_wdata_o_wires := RS2_Data_IE(7 downto 0) & std_logic_vector(to_unsigned(0, 24));
                  when others =>
                    null;
                end case;
                data_req_o_wires       := '1';
                data_be_internal_wires := "1111";
                if(data_addr_internal_wires(1 downto 0) = "00") then
                  if (load_err = '1') then
                    nextstate_IE_wires         := normal;  -- was except_wait_state;
                    set_except_condition_wires := '1';
                  elsif (store_err = '1') then
                    nextstate_IE_wires         := normal;  -- was except_wait_state;
                    set_except_condition_wires := '1';
                  elsif data_gnt_i = '1' then
                    nextstate_IE_wires := data_valid_waiting;
                    busy_IE_wires      := '1';
                  else
                    nextstate_IE_wires := data_grant_waiting;
                    busy_IE_wires      := '1';
                  end if;
                else
                  set_except_condition_wires := '1';
                  busy_IE_wires              := '1';
                end if;
              end if;
            end if;

            if decoded_instruction_IE(SH_bit_position) = '1' then
              data_we_o_wires := '1';   -- is a writing
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(S_immediate(instr_word_IE)));
              --data_wdata_o_wires         <= RS2_Data_IE;  GG it is not so easy
              case data_addr_internal_wires(1 downto 0) is
                when "00" =>
                  data_wdata_o_wires := RS2_Data_IE(31 downto 0);
                when "01" =>
                  data_wdata_o_wires := RS2_Data_IE(23 downto 0) & std_logic_vector(to_unsigned(0, 8));
                when "10" =>
                  data_wdata_o_wires := RS2_Data_IE(15 downto 0) & std_logic_vector(to_unsigned(0, 16));
                when "11" =>
                  data_wdata_o_wires := RS2_Data_IE(7 downto 0) & std_logic_vector(to_unsigned(0, 24));
                when others =>
                  null;
              end case;
              data_req_o_wires       := '1';
              data_be_internal_wires := "0011";
              if(data_addr_internal_wires(0) = '0') then
                if (load_err = '1') then
                  nextstate_IE_wires         := normal;  -- was except_wait_state;
                  set_except_condition_wires := '1';
                elsif (store_err = '1') then
                  nextstate_IE_wires         := normal;  -- was except_wait_state;
                  set_except_condition_wires := '1';
                elsif data_gnt_i = '1' then
                  nextstate_IE_wires := data_valid_waiting;
                  busy_IE_wires      := '1';
                else
                  nextstate_IE_wires := data_grant_waiting;
                  busy_IE_wires      := '1';
                end if;
              else
                set_except_condition_wires := '1';
                busy_IE_wires              := '1';
              end if;
            end if;

            if decoded_instruction_IE(SB_bit_position) = '1' then
              data_we_o_wires := '1';   -- is a writing
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(S_immediate(instr_word_IE)));
              --data_wdata_o_wires         <= RS2_Data_IE;  GG it is not so easy
              case data_addr_internal_wires(1 downto 0) is
                when "00" =>
                  data_wdata_o_wires := RS2_Data_IE(31 downto 0);
                when "01" =>
                  data_wdata_o_wires := RS2_Data_IE(23 downto 0) & std_logic_vector(to_unsigned(0, 8));
                when "10" =>
                  data_wdata_o_wires := RS2_Data_IE(15 downto 0) & std_logic_vector(to_unsigned(0, 16));
                when "11" =>
                  data_wdata_o_wires := RS2_Data_IE(7 downto 0) & std_logic_vector(to_unsigned(0, 24));
                when others =>
                  null;
              end case;
              data_req_o_wires       := '1';
              data_be_internal_wires := "0001";
              if (load_err = '1') then
                nextstate_IE_wires         := normal;  -- was except_wait_state;
                set_except_condition_wires := '1';
              elsif (store_err = '1') then
                nextstate_IE_wires         := normal;  -- was except_wait_state;
                set_except_condition_wires := '1';
              elsif data_gnt_i = '1' then
                nextstate_IE_wires := data_valid_waiting;
                busy_IE_wires      := '1';
              else
                nextstate_IE_wires := data_grant_waiting;
                busy_IE_wires      := '1';
              end if;
            end if;

            if decoded_instruction_IE(CSRRW_bit_position) = '1' or decoded_instruction_IE(CSRRWI_bit_position) = '1' then
              nextstate_IE_wires := csr_instr_wait_state;
              busy_IE_wires      := '1';
            end if;

            if decoded_instruction_IE(CSRRC_bit_position) = '1' or decoded_instruction_IE(CSRRCI_bit_position) = '1'
              or decoded_instruction_IE(CSRRS_bit_position) = '1' or decoded_instruction_IE(CSRRSI_bit_position) = '1' then
              nextstate_IE_wires := csr_instr_wait_state;
              busy_IE_wires      := '1';
            end if;

            if decoded_instruction_IE(ECALL_bit_position) = '1' then
              nextstate_IE_wires         := normal;  -- was except_wait_state;
              set_except_condition_wires := '1';
            end if;

            if decoded_instruction_IE(EBREAK_bit_position) = '1' then
              ebreak_instr_wires := '1';
              nextstate_IE_wires := normal;
            end if;

            if decoded_instruction_IE(MRET_bit_position) = '1' then
              set_mret_condition_wires := '1';
              if fetch_enable_i = '0' then
                nextstate_IE_wires := sleep;
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(ERET_bit_position) = '1' then
              set_mret_condition_wires := '1';
              if fetch_enable_i = '0' then
                nextstate_IE_wires := sleep;
              else
                nextstate_IE_wires := normal;
              end if;
            end if;

            if decoded_instruction_IE(WFI_bit_position) = '1' then
              set_wfi_condition  <= '1';
              nextstate_IE_wires := normal;
            end if;

            if decoded_instruction_IE(AMOSWAP_bit_position) = '1' then
              amo_instr_wires := '1';
              if amoswap_writeback_lat = '0' and amo_load_skip = '0' then  -- Step 1: Load the address in register RS1 from the memory into RD
                data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
                data_be_internal_wires   := "1111";
                data_req_o_wires         := '1';
                data_we_o_wires          := '0';
                if(data_addr_internal_wires(1 downto 0) = "00") then
                  if (load_err = '1') then
                    nextstate_IE_wires         := normal;
                    set_except_condition_wires := '1';
                  elsif (store_err = '1') then
                    nextstate_IE_wires         := normal;
                    set_except_condition_wires := '1';
                  elsif data_gnt_i = '1' then
                    nextstate_IE_wires := data_valid_waiting;
                    busy_IE_wires      := '1';
                  else
                    nextstate_IE_wires := data_grant_waiting;
                    busy_IE_wires      := '1';
                  end if;
                else
                  set_except_condition_wires := '1';
                  busy_IE_wires              := '1';
                end if;
              elsif amoswap_writeback_lat = '1' or amo_load_skip = '1' then  -- Step 2: Swap Instructions RS2 and RD
                data_we_o_wires          := '1';           -- is a writing
                data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
                data_wdata_o_wires       := RS2_Data_IE(31 downto 0);
                data_req_o_wires         := '1';
                data_be_internal_wires   := "1111";
                if(data_addr_internal_wires(1 downto 0) = "00") then
                  if (load_err = '1') then
                    nextstate_IE_wires         := normal;  -- was except_wait_state;
                    set_except_condition_wires := '1';
                  elsif (store_err = '1') then
                    nextstate_IE_wires         := normal;  -- was except_wait_state;
                    set_except_condition_wires := '1';
                  elsif data_gnt_i = '1' then
                    nextstate_IE_wires := data_valid_waiting;
                    busy_IE_wires      := '1';
                  else
                    nextstate_IE_wires := data_grant_waiting;
                    busy_IE_wires      := '1';
                  end if;
                else
                  set_except_condition_wires := '1';
                  busy_IE_wires              := '1';
                end if;
              end if;
            end if;

            if decoded_instruction_IE(ILL_bit_position) = '1' then  -- ILLEGAL_INSTRUCTION
              nextstate_IE_wires         := normal;  -- was except_wait_state;
              set_except_condition_wires := '1';
            end if;

            if decoded_instruction_IE(NOP_bit_position) = '1' then
              nextstate_IE_wires := normal;  -- was except_wait_state;
            end if;

          -- EXECUTE OF INSTRUCTION (END)
          end if;  -- instr_rvalid_IE values 

        when data_grant_waiting =>
          data_req_o_wires := '1';
          if data_we_o_lat = '1' then
            if amo_instr_lat = '0' then
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(S_immediate(instr_word_IE)));
              data_wdata_o_wires := RD_Data_IE(31 downto 0);
            else
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
            end if;
          else
            if amo_instr_lat = '0' then
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE) + signed(I_immediate(instr_word_IE)));
            else
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
            end if;
          end if;

          if (load_err = '1') then
            nextstate_IE_wires         := normal;  -- was except_wait_state;
            set_except_condition_wires := '1';
          elsif (store_err = '1') then
            nextstate_IE_wires         := normal;  -- was except_wait_state;
            set_except_condition_wires := '1';
          elsif data_gnt_i = '1' then
            nextstate_IE_wires := data_valid_waiting;
            busy_IE_wires      := '1';
          else
            nextstate_IE_wires := data_grant_waiting;
            busy_IE_wires      := '1';
          end if;

        when data_valid_waiting =>

          if data_we_o_lat = '1' then
            if amo_instr_lat = '0' then
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE)
                                                           + signed(S_immediate(instr_word_IE)));
              data_wdata_o_wires := RD_Data_IE(31 downto 0);
            else
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
            end if;
          else
            if amo_instr_lat = '0' then
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE) + signed(I_immediate(instr_word_IE)));
            else
              data_addr_internal_wires := std_logic_vector(signed(RS1_Data_IE));
            end if;
          end if;

          if (load_err = '1') then
            nextstate_IE_wires         := normal;  -- was except_wait_state;
            set_except_condition_wires := '1';
          elsif (store_err = '1') then
            nextstate_IE_wires         := normal;  -- was except_wait_state;
            set_except_condition_wires := '1';
          elsif data_rvalid_i = '1' then
            if dbg_req_o = '1' then
              nextstate_IE_wires := debug;
              dbg_ack_i_wires    := '1';
              busy_IE_wires      := '1';
            else
              nextstate_IE_wires := normal;
              if amo_instr_lat = '1' and data_we_o_lat = '0' then  --GG
                amoswap_writeback_wires := '1';
                busy_IE_wires           := '1';
              end if;
            end if;
          else
            nextstate_IE_wires := data_valid_waiting;
            busy_IE_wires      := '1';
          end if;

        when csr_instr_wait_state =>
          if (csr_instr_done = '0') then
            nextstate_IE_wires := csr_instr_wait_state;
            busy_IE_wires      := '1';
          elsif (csr_instr_done = '1' and csr_access_denied_o = '1') then  -- ILLEGAL_INSTRUCTION
            nextstate_IE_wires         := normal;
            set_except_condition_wires := '1';
          else
            nextstate_IE_wires := normal;
          end if;

      end case;  -- fsm_IE state cases
    end if;  -- refers to reset signal

    PC_offset                  <= PC_offset_wires;
    data_addr_internal         <= data_addr_internal_wires;
    data_wdata_o               <= data_wdata_o_wires;
    data_be_internal           <= data_be_internal_wires;
    data_we_o                  <= data_we_o_wires;
    absolute_jump              <= absolute_jump_wires;
    busy_IE                    <= busy_IE_wires;
    set_except_condition       <= set_except_condition_wires;
    set_branch_condition       <= set_branch_condition_wires;
    set_mret_condition         <= set_mret_condition_wires;
    jump_instr                 <= jump_instr_wires;
    branch_instr               <= branch_instr_wires;
    ebreak_instr               <= ebreak_instr_wires;
    amo_instr                  <= amo_instr_wires;
    amoswap_writeback          <= amoswap_writeback_wires;
    dbg_ack_i                  <= dbg_ack_i_wires;
    nextstate_IE               <= nextstate_IE_wires;
    data_valid_waiting_counter <= data_valid_waiting_counter_wires;
    data_req_o                 <= data_req_o_wires;
  end process;

  fsm_IE_state : process(clk_i, rst_ni, fetch_enable_i)
  begin
    if rst_ni = '0' then
      data_we_o_lat         <= '0';
      amo_instr_lat         <= '0';
      amoswap_writeback_lat <= '0';
      if fetch_enable_i = '1' then
        state_IE <= normal;
      else
        state_IE <= reset;
      end if;
    elsif rising_edge(clk_i) then
      data_we_o_lat         <= data_we_o;
      amo_instr_lat         <= amo_instr;
      amoswap_writeback_lat <= amoswap_writeback;
      state_IE              <= nextstate_IE;
    end if;
  end process;
-------------------------------------------------------------------end of IE stage -----------------
----------------------------------------------------------------------------------------------------

-----------------------------------------------------------------------------------------------------
-- Stage WB - (WRITEBACK)
-----------------------------------------------------------------------------------------------------
-- Writes back on register file
-----------------------------------------------------------------------------------------------------

  fsm_WB_seq : process(clk_i, rst_ni)

  begin
    if rst_ni = '0' then
      for index in 0 to 31
      loop
        for h in harc_range loop
          regfile(h)(index) <= std_logic_vector(to_unsigned(0, 32));
        end loop;
      end loop;
    elsif rising_edge(clk_i) then
      if instr_rvalid_WB = '1' and WB_RD_EN = '1' then
        regfile(harc_WB)(rd(instr_word_WB)) <= WB_RD(harc_WB);
      end if;

      if instr_rvalid_WB = '1' and WB_RS2_EN = '1' then
        regfile(harc_WB)(rs2(instr_word_WB)) <= WB_RS2(harc_WB);
      end if;
    end if;
  end process;


----------------------------------------------------------------------------------------------------
-- Control and Status Units -- 
----------------------------------------------------------------------------------------------------
-- Manages CSR instructions and CSR automatic updates following traps and special instructions.
-- Note: in the present version, gives priority to CSR automatic updates over CSR instr. execution
-- Note: the CSRegisters are replicated along with the related logic, as in the pc update logic,
-- because events coming from different sources may require to update the CSRs concurrently.
----------------------------------------------------------------------------------------------------

  -- here we start replicating the logic ------------------------------------------------------
  CSR_updating_logic : for h in harc_range generate

    -- hardwired read-only connections  
    -- note: MCPUID, MIMPID, MHARTID replicated only for easy coding, they return same value for all threads
    MCPUID(h) <= std_logic_vector(to_unsigned(256, 32));  -- xx move init value in pkg
    MIMPID(h) <= std_logic_vector(to_unsigned(32768, 32));  -- xx move init value in pkg
    MHARTID(h) <= std_logic_vector(resize(unsigned(cluster_id_i) &
                                          to_unsigned(harc_IE, THREAD_ID_SIZE), 32));

    -- irq request vector shifted by 2 bits, used in interrupt handler routine
    MIRQ(h) <= "0000000000000000000000000" & irq_id_i & "00";

    csr_instr_req_replicated(h) <= '1' when csr_instr_req = '1' and harc_to_csr = h else '0';

    CSR_unit_op : process(clk_i, rst_ni)  -- single cycle unit, one process, fully synchronous 

      variable MIP_3_wire  : replicated_bit;
      
    begin

      if rst_ni = '0' then
        MSTATUS(h)  <= MSTATUS_RESET_VALUE;
        MESTATUS(h) <= MESTATUS_RESET_VALUE;
        MEPC(h)     <= MEPC_RESET_VALUE;
        MCAUSE(h)   <= MCAUSE_RESET_VALUE;
        MTVEC(h)    <= MTVEC_RESET_VALUE(h);
        PCER(h)     <= PCER_RESET_VALUE(h);
        --Reset of counters and related registers
        MCYCLE(h)                         <= x"00000000";
        MINSTRET(h)                       <= x"00000000";
        MHPMCOUNTER3(h)                   <= x"00000000";
        --MHPMCOUNTER4(h)       <= x"00000000"; -- xxxxxxxxxxxx why commented? (ask Cerutti)
        --MHPMCOUNTER5(h)       <= x"00000000";
        MHPMCOUNTER6(h)                   <= x"00000000";
        MHPMCOUNTER7(h)                   <= x"00000000";
        MHPMCOUNTER8(h)                   <= x"00000000";
        MHPMCOUNTER9(h)                   <= x"00000000";
        MHPMCOUNTER10(h)                  <= x"00000000";
        MCYCLEH(h)                        <= x"00000000";
        MINSTRETH(h)                      <= x"00000000";
        MHPMEVENT3(h)                     <= PCER_RESET_VALUE(h)(2);
        --MHPMEVENT4(h) <=PCER_RESET_VALUE(3);  -- xxxxxxxxxxxxxxx as above
        --MHPMEVENT5(h) <=PCER_RESET_VALUE(4);  
        MHPMEVENT6(h)                     <= PCER_RESET_VALUE(h)(5);
        MHPMEVENT7(h)                     <= PCER_RESET_VALUE(h)(6);
        MHPMEVENT8(h)                     <= PCER_RESET_VALUE(h)(7);
        MHPMEVENT9(h)                     <= PCER_RESET_VALUE(h)(8);
        MHPMEVENT10(h)                    <= PCER_RESET_VALUE(h)(9);
        MIP(h)                            <= MIP_RESET_VALUE;
        -- Note: what we actually need is: 
        --  MIP_wires(h)(3) <= '0'; -- this will be made accessible by memory map, sw interrupt
        --  MIP_wires(h)(7) <= '0'; -- 3,7,11 are the only MIP_wires bits used  
        --  MIP_wires(h)(11) <= '0'; -- anyway we have to initialize the full vector in vhdl
        --PCCRs tbd xxxxxxxxxxxxxxxx???
        --PCER tbd xxxxxxxxxxxxxxxxx???
        --PCMR tbd  xxxxxxxxxxxxxx ???
        csr_instr_done_replicated(h)      <= '0';
        csr_access_denied_o_replicated(h) <= '0';
        csr_rdata_o_replicated(h)         <= (others => '0');

      elsif rising_edge(clk_i) then
        MIP_3_wire(h) := '0';
        -- NOTE: PRIV ISA Manual v1.9.1, decreasing priority order: 
        --       ext. int., sw int., timer int., exceptions.
        -- MSTATUS(3) is the MIE bit, machine mode interrupt enable.
        -- CSR updating for all possibe sources follows.

        --  Interrupt-caused CSR updating  ---------------------------------
        -- note: PC just udpdated, MIP_wires can't have been cleared yet.
        if served_irq(h) = '1' and MIP(h)(11) = '1' then
          -- it is the MEIP bit, ext. irq
          MCAUSE(h)     <= "1" & std_logic_vector(to_unsigned(11, 31));  -- ext. irq
          MESTATUS(h)   <= MSTATUS(h);
          MEPC(h)       <= pc_IE;  -- xxxx this is to be checked, may be pc_IE + 4?
          MSTATUS(h)(3) <= '0';         -- interrupt disabled 
        elsif served_irq(h) = '1' and MIP(h)(3) = '1' then
          -- it is the MSIP bit, sw interrupt req
          MCAUSE(h)     <= "1" & std_logic_vector(to_unsigned(3, 31));  -- sw interrupt
          MESTATUS(h)   <= MSTATUS(h);
          MEPC(h)       <= pc_IE;  -- xxx this is to be checked, may be pc_IE + 4?
          MSTATUS(h)(3) <= '0';         -- interrupt disabled               
        elsif served_irq(h) = '1' and MIP(h)(7) = '1' then
          -- it is the MSIP bit, timer interrupt req
          MCAUSE(h)     <= "1" & std_logic_vector(to_unsigned(7, 31));  -- timer interrupt
          MESTATUS(h)   <= MSTATUS(h);
          MEPC(h)       <= pc_IE;  -- xxxx this is to be checked, may be pc_IE + 4?
          MSTATUS(h)(3) <= '0';         -- interrupt disabled               
        --  Exception-caused CSR updating ----------------------------------
        elsif served_except_condition(h) = '1' then
          MCAUSE(h)     <= csr_wdata_i;  -- passed from IE stage
          MESTATUS(h)   <= MSTATUS(h);
          MEPC(h)       <= pc_except_value(h);
          MSTATUS(h)(3) <= '0';  -- interrupt disabled, xx this is to be checked           
          if (csr_wdata_i = LOAD_MISALIGNED_EXCEPT_CODE or csr_wdata_i = STORE_MISALIGNED_EXCEPT_CODE) then
            MBADADDR(h) <= data_addr_internal;
          end if;
        -- mret-caused CSR updating ----------------------------------------
        elsif served_mret_condition(h) then
          MSTATUS(h) <= MESTATUS(h);
        -- CSR instruction handling ----------------------------------------      
        elsif(csr_instr_done_replicated(h) = '1') then
          csr_instr_done_replicated(h)      <= '0';
          csr_access_denied_o_replicated(h) <= '0';
        elsif csr_instr_req_replicated(h) = '1' then
          csr_instr_done_replicated(h) <= '1';
          if (csr_op_i /= "000" and csr_op_i /= "100") then  -- check for valid operation 
            case csr_addr_i is
              when MSTATUS_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MSTATUS(h);
                    MSTATUS(h)(0) <= csr_wdata_i(0);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MSTATUS(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MSTATUS(h)(0) <= (MSTATUS(h)(0) or csr_wdata_i(0));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MSTATUS(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MSTATUS(h)(0) <= MSTATUS(h)(0) and (not csr_wdata_i(0));
                    end if;
                  when others =>
                    null;
                end case;
              when MIP_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (11 => MIP(h)(11), 7 => MIP(h)(7), 3 => MIP(h)(3), others => '0');
                    MIP_3_wire(h) := csr_wdata_i(3);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MSTATUS(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MIP_3_wire(h) := (MIP(h)(3) or csr_wdata_i(3));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (11 => MIP(h)(11), 7 => MIP(h)(7), 3 => MIP(h)(3), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MIP_3_wire(h) :=  (MIP(h)(3) and (not csr_wdata_i(3)));
                    end if;
                  when others =>
                    null;
                end case;
              when MEPC_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MEPC(h);
                    MEPC(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MEPC(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MEPC(h) <= (MEPC(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MEPC(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MEPC(h) <= (MEPC(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;
              when MTVEC_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MTVEC(h);
                    MEPC(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MTVEC(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MTVEC(h) <= (MTVEC(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MTVEC(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MTVEC(h) <= (MTVEC(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;
              when MCAUSE_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MCAUSE(h);
                    MCAUSE(h)(31)         <= csr_wdata_i(31);
                    MCAUSE(h)(4 downto 0) <= csr_wdata_i(4 downto 0);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MCAUSE(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCAUSE(h)(31)         <= (MCAUSE(h)(31) or csr_wdata_i(31));
                      MCAUSE(h)(4 downto 0) <= (MCAUSE(h)(4 downto 0) or csr_wdata_i(4 downto 0));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MCAUSE(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCAUSE(h)(4 downto 0) <= (MCAUSE(h)(4 downto 0) and not(csr_wdata_i(4 downto 0)));
                      MCAUSE(h)(31)         <= (MCAUSE(h)(31) and not(csr_wdata_i(31)));
                    end if;
                  when others =>
                    null;
                end case;
              when MESTATUS_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MESTATUS(h);
                    MESTATUS(h)(0) <= csr_wdata_i(0);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MESTATUS(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MESTATUS(h)(0) <= (MESTATUS(h)(0) or csr_wdata_i(0));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MESTATUS(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MESTATUS(h)(0) <= (MESTATUS(h)(0) and (not csr_wdata_i(0)));
                    end if;
                  when others =>
                    null;
                end case;
              when MCPUID_addr =>       -- read only
                case csr_op_i is
                  when CSRRC|CSRRS|CSRRCI|CSRRSI =>
                    if(rs1(instr_word_IE) = 0) then
                      csr_rdata_o_replicated(h) <= MCPUID(h);
                    else
                      csr_access_denied_o_replicated(h) <= '1';
                    end if;
                  when CSRRW|CSRRWI =>
                    csr_access_denied_o_replicated(h) <= '1';
                  when others =>
                    null;
                end case;
              when MIMPID_addr =>       -- read only
                case csr_op_i is
                  when CSRRC|CSRRS|CSRRCI|CSRRSI =>
                    if(rs1(instr_word_IE) = 0) then
                      csr_rdata_o_replicated(h) <= MIMPID(h);
                    else
                      csr_access_denied_o_replicated(h) <= '1';
                    end if;
                  when CSRRW|CSRRWI =>
                    csr_access_denied_o_replicated(h) <= '1';
                  when others =>
                    null;
                end case;
              when MHARTID_addr =>      -- read only
                case csr_op_i is
                  when CSRRC|CSRRS|CSRRCI|CSRRSI =>
                    if(rs1(instr_word_IE) = 0) then
                      csr_rdata_o_replicated(h) <= MHARTID(h);
                    else
                      csr_access_denied_o_replicated(h) <= '1';
                    end if;
                  when CSRRW|CSRRWI =>
                    csr_access_denied_o_replicated(h) <= '1';
                  when others =>
                    null;
                end case;
              when MIRQ_addr =>         -- read only
                case csr_op_i is
                  when CSRRC|CSRRS|CSRRCI|CSRRSI =>
                    if(rs1(instr_word_IE) = 0) then
                      csr_rdata_o_replicated(h) <= MIRQ(h);
                    else
                      csr_access_denied_o_replicated(h) <= '1';
                    end if;
                  when CSRRW|CSRRWI =>
                    csr_access_denied_o_replicated(h) <= '1';
                  when others =>
                    null;
                end case;
              when BADADDR_addr =>      -- read only
                case csr_op_i is
                  when CSRRC|CSRRS|CSRRCI|CSRRSI =>
                    if(rs1(instr_word_IE) = 0) then
                      csr_rdata_o_replicated(h) <= MBADADDR(h);
                    else
                      csr_access_denied_o_replicated(h) <= '1';
                    end if;
                  when CSRRW|CSRRWI =>
                    csr_access_denied_o_replicated(h) <= '1';
                  when others =>
                    null;
                end case;

              when MCYCLE_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MCYCLE(h);
                    MCYCLE(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MCYCLE(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCYCLE(h) <= (MCYCLE(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MCYCLE(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCYCLE(h) <= (MCYCLE(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MINSTRET_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRET(h))-1);  --old value in reading
                    MINSTRET(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRET(h))-1);
                    if(rs1(instr_word_IE) /= 0) then
                      MINSTRET(h) <= (MINSTRET(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRET(h))-1);
                    if(rs1(instr_word_IE) /= 0) then
                      MINSTRET(h) <= (MINSTRET(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MCYCLEH_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MCYCLEH(h);
                    MCYCLEH(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MCYCLEH(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCYCLEH(h) <= (MCYCLEH(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MCYCLEH(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MCYCLEH(h) <= (MCYCLEH(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MINSTRETH_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      if(MINSTRET(h) = x"00000000" and MINSTRETH(h) /= x"00000000") then
                        csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRETH(h))-1);
                      else
                        csr_rdata_o_replicated(h) <= MINSTRETH(h);
                      end if;
                      MINSTRETH(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    if(MINSTRET(h) = x"00000000" and MINSTRETH(h) /= x"00000000") then
                      csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRETH(h))-1);
                    else
                      csr_rdata_o_replicated(h) <= MINSTRETH(h);
                    end if;
                    if(rs1(instr_word_IE) /= 0) then
                      MINSTRETH(h) <= (MINSTRETH(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    if(MINSTRET(h) = x"00000000" and MINSTRETH(h) /= x"00000000") then
                      csr_rdata_o_replicated(h) <= std_logic_vector(unsigned(MINSTRETH(h))-1);
                    else
                      csr_rdata_o_replicated(h) <= MINSTRETH(h);
                    end if;
                    if(rs1(instr_word_IE) /= 0) then
                      MINSTRETH(h) <= (MINSTRETH(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMCOUNTER3_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER3(h);
                    MHPMCOUNTER3(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER3(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER3(h) <= (MHPMCOUNTER3(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER3(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER3(h) <= (MHPMCOUNTER3(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

                -- when MHPMCOUNTER4_addr =>
                -- case csr_op_i is
                -- when CSRRW|CSRRWI =>
                -- if (rd(instr_word_IE) /= 0) then
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER4(h);
                -- end if;
                -- MHPMCOUNTER4(h) <= csr_wdata_i;
                -- when CSRRS|CSRRSI =>
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER4(h);
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMCOUNTER4(h) <= (MHPMCOUNTER4(h) or csr_wdata_i);
                -- end if;
                -- when CSRRC|CSRRCI =>
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER4(h);
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMCOUNTER4(h) <= (MHPMCOUNTER4(h) and not(csr_wdata_i));
                -- end if;
                -- when others =>
                -- null;
                -- end case;

                -- when MHPMCOUNTER5_addr =>
                -- case csr_op_i is
                -- when CSRRW|CSRRWI =>
                -- if (rd(instr_word_IE) /= 0) then
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER5(h);
                -- end if;
                -- MHPMCOUNTER5(h) <= csr_wdata_i;
                -- when CSRRS|CSRRSI =>
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER5(h);
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMCOUNTER5(h) <= (MHPMCOUNTER5(h) or csr_wdata_i);
                -- end if;
                -- when CSRRC|CSRRCI =>
                -- csr_rdata_o_replicated(h) <= MHPMCOUNTER5(h);
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMCOUNTER5(h) <= (MHPMCOUNTER5(h) and not(csr_wdata_i));
                -- end if;
                -- when others =>
                -- null;
                -- end case;

              when MHPMCOUNTER6_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER6(h);
                    MHPMCOUNTER6(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER6(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER6(h) <= (MHPMCOUNTER6(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER6(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER6(h) <= (MHPMCOUNTER6(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMCOUNTER7_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER7(h);
                    MHPMCOUNTER7(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER7(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER7(h) <= (MHPMCOUNTER7(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER7(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER7(h) <= (MHPMCOUNTER7(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMCOUNTER8_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER8(h);
                    MHPMCOUNTER8(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER8(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER8(h) <= (MHPMCOUNTER8(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER8(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER8(h) <= (MHPMCOUNTER8(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMCOUNTER9_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER9(h);
                    MHPMCOUNTER9(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER9(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER9(h) <= (MHPMCOUNTER9(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER9(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER9(h) <= (MHPMCOUNTER9(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMCOUNTER10_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= MHPMCOUNTER10(h);
                    MHPMCOUNTER10(h) <= csr_wdata_i;
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER10(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER10(h) <= (MHPMCOUNTER10(h) or csr_wdata_i);
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= MHPMCOUNTER10(h);
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMCOUNTER10(h) <= (MHPMCOUNTER10(h) and not(csr_wdata_i));
                    end if;
                  when others =>
                    null;
                end case;


              when PCER_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= PCER(h);
                    PCER(h)        <= csr_wdata_i;
                    MHPMEVENT3(h)  <= csr_wdata_i(2);
                    --MHPMEVENT4(h) <= csr_wdata_i(3);
                    --MHPMEVENT5(h) <= csr_wdata_i(4);  
                    MHPMEVENT6(h)  <= csr_wdata_i(5);
                    MHPMEVENT7(h)  <= csr_wdata_i(6);
                    MHPMEVENT8(h)  <= csr_wdata_i(7);
                    MHPMEVENT9(h)  <= csr_wdata_i(8);
                    MHPMEVENT10(h) <= csr_wdata_i(9);

                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= PCER(h);
                    if(rs1(instr_word_IE) /= 0) then
                      PCER(h)        <= (PCER(h) or csr_wdata_i);
                      MHPMEVENT3(h)  <= (PCER(h)(2) or csr_wdata_i(2));
                                        --MHPMEVENT(h)4 <= (PCER(h)(3) or csr_wdata_i(3));      
                                        --MHPMEVENT(h)5 <= (PCER(h)(4) or csr_wdata_i(4));
                      MHPMEVENT6(h)  <= (PCER(h)(5) or csr_wdata_i(5));
                      MHPMEVENT7(h)  <= (PCER(h)(6) or csr_wdata_i(6));
                      MHPMEVENT8(h)  <= (PCER(h)(7) or csr_wdata_i(7));
                      MHPMEVENT9(h)  <= (PCER(h)(8) or csr_wdata_i(8));
                      MHPMEVENT10(h) <= (PCER(h)(9) or csr_wdata_i(9));

                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= PCER(h);
                    if(rs1(instr_word_IE) /= 0) then
                      PCER(h)        <= (PCER(h) and not(csr_wdata_i));
                      MHPMEVENT3(h)  <= (PCER(h)(2) and not (csr_wdata_i(2)));
                                        --MHPMEVENT4(h) <= (PCER(h)(3) and not (csr_wdata_i(3)));       
                                        --MHPMEVENT5 <= (PCER(h)(4) and not (csr_wdata_i(4)));
                      MHPMEVENT6(h)  <= (PCER(h)(5) and not (csr_wdata_i(5)));
                      MHPMEVENT7(h)  <= (PCER(h)(6) and not (csr_wdata_i(6)));
                      MHPMEVENT8(h)  <= (PCER(h)(7) and not (csr_wdata_i(7)));
                      MHPMEVENT9(h)  <= (PCER(h)(8) and not (csr_wdata_i(8)));
                      MHPMEVENT10(h) <= (PCER(h)(9) and not (csr_wdata_i(9)));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMEVENT3_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                     csr_rdata_o_replicated(h) <= (2 => MHPMEVENT3(h), others => '0');
                    MHPMEVENT3(h) <= csr_wdata_i(2);
                    PCER(h)(2)    <= csr_wdata_i(2);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (2 => MHPMEVENT3(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT3(h) <= (MHPMEVENT3(h) or csr_wdata_i(2));
                      PCER(h)(2)    <= (PCER(h)(2) or csr_wdata_i(2));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (2 => MHPMEVENT3(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT3(h) <= (MHPMEVENT3(h) and not(csr_wdata_i(2)));
                    end if;
                  when others =>
                    null;
                end case;

                -- when MHPMEVENT4_addr => -- xxxxxxxxxxxxxx why are these cases commented????????? (ask Cerutti)
                -- case csr_op_i is
                -- when CSRRW|CSRRWI =>
                -- if (rd(instr_word_IE) /= 0) then
                -- csr_rdata_o_replicated(h) <= (3 => MHPMEVENT4(h), others => '0');
                -- end if;
                -- MHPMEVENT4(h) <= csr_wdata_i(3);
                -- PCER(h)(3) <= csr_wdata_i(3);
                -- when CSRRS|CSRRSI =>
                -- csr_rdata_o_replicated(h) <= (3 => MHPMEVENT4(h), others => '0');
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMEVENT4(h) <= (MHPMEVENT4(h) or csr_wdata_i(3));
                                -- PCER(h)(3) <= (PCER(h)(3) or csr_wdata_i(3));
                -- end if;
                -- when CSRRC|CSRRCI =>
                -- csr_rdata_o_replicated(h) <= (3 => MHPMEVENT4, others => '0');
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMEVENT4(h) <= (MHPMEVENT4(h) and not(csr_wdata_i(3)));
                -- end if;
                -- when others =>
                -- null;
                -- end case;

                -- when MHPMEVENT5_addr =>
                -- case csr_op_i is
                -- when CSRRW|CSRRWI =>
                -- if (rd(instr_word_IE) /= 0) then
                -- csr_rdata_o_replicated(h) <= (4 => MHPMEVENT5(h), others => '0');
                -- end if;
                -- MHPMEVENT5(h) <= csr_wdata_i(4);
                -- PCER(h)(4) <= csr_wdata_i(4);
                -- when CSRRS|CSRRSI =>
                -- csr_rdata_o_replicated(h) <= (4 => MHPMEVENT5(h), others => '0');
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMEVENT5(h) <= (MHPMEVENT5(h) or csr_wdata_i(4));
                                -- PCER(h)(4) <= (PCER(h)(4) or csr_wdata_i(4));
                -- end if;
                -- when CSRRC|CSRRCI =>
                -- csr_rdata_o_replicated(h) <= (4 => MHPMEVENT5(h), others => '0');
                -- if(rs1(instr_word_IE) /= 0) then
                -- MHPMEVENT5(h) <= (MHPMEVENT5(h) and not(csr_wdata_i(4)));
                -- end if;
                -- when others =>
                -- null;
                -- end case;

              when MHPMEVENT6_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (5 => MHPMEVENT6(h), others => '0');
                    MHPMEVENT6(h) <= csr_wdata_i(5);
                    PCER(h)(5)    <= csr_wdata_i(5);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (5 => MHPMEVENT6(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT6(h) <= (MHPMEVENT6(h) or csr_wdata_i(5));
                      PCER(h)(5)    <= (PCER(h)(5) or csr_wdata_i(5));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (5 => MHPMEVENT6(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT6(h) <= (MHPMEVENT6(h) and not(csr_wdata_i(5)));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMEVENT7_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (6 => MHPMEVENT7(h), others => '0');
                    MHPMEVENT7(h) <= csr_wdata_i(6);
                    PCER(h)(6)    <= csr_wdata_i(6);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (6 => MHPMEVENT7(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT7(h) <= (MHPMEVENT7(h) or csr_wdata_i(6));
                      PCER(h)(6)    <= (PCER(h)(6) or csr_wdata_i(6));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (6 => MHPMEVENT7(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT7(h) <= (MHPMEVENT7(h) and not(csr_wdata_i(6)));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMEVENT8_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (7 => MHPMEVENT8(h), others => '0');
                    MHPMEVENT8(h) <= csr_wdata_i(7);
                    PCER(h)(7)    <= csr_wdata_i(7);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (7 => MHPMEVENT8(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT8(h) <= (MHPMEVENT8(h) or csr_wdata_i(7));
                      PCER(h)(7)    <= (PCER(h)(7) or csr_wdata_i(7));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (7 => MHPMEVENT8(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT8(h) <= (MHPMEVENT8(h) and not(csr_wdata_i(7)));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMEVENT9_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (8 => MHPMEVENT9(h), others => '0');
                    MHPMEVENT9(h) <= csr_wdata_i(8);
                    PCER(h)(8)    <= csr_wdata_i(8);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (8 => MHPMEVENT9(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT9(h) <= (MHPMEVENT9(h) or csr_wdata_i(8));
                      PCER(h)(8)    <= (PCER(h)(8) or csr_wdata_i(8));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (8 => MHPMEVENT9(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT9(h) <= (MHPMEVENT9(h) and not(csr_wdata_i(8)));
                    end if;
                  when others =>
                    null;
                end case;

              when MHPMEVENT10_addr =>
                case csr_op_i is
                  when CSRRW|CSRRWI =>
                      csr_rdata_o_replicated(h) <= (9 => MHPMEVENT10(h), others => '0');
                    MHPMEVENT10(h) <= csr_wdata_i(9);
                    PCER(h)(9)     <= csr_wdata_i(9);
                  when CSRRS|CSRRSI =>
                    csr_rdata_o_replicated(h) <= (9 => MHPMEVENT10(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT10(h) <= (MHPMEVENT10(h) or csr_wdata_i(9));
                      PCER(h)(9)     <= (PCER(h)(9) or csr_wdata_i(9));
                    end if;
                  when CSRRC|CSRRCI =>
                    csr_rdata_o_replicated(h) <= (9 => MHPMEVENT10(h), others => '0');
                    if(rs1(instr_word_IE) /= 0) then
                      MHPMEVENT10(h) <= (MHPMEVENT10(h) and not(csr_wdata_i(9)));
                    end if;
                  when others =>
                    null;
                end case;

              when others => -- invalid CSR address. ignored. May raise exception in future.
                csr_rdata_o_replicated(h) <= (others => '0');  -- unhandled situation
                                                               -- default value
            end case;
          else
            null;  -- invalid CSR operation, ignored. May raise exception in future.
          end if;  -- csr_op_i
        end if;  -- trap conditions, csr_instr_done, csr_instr_req

        -- PERFORMANCE COUNTER AUTOMATIC UPDATING --

        if dbg_req_o = '0' then
          --THIS BIG CONDITION CHECKS WRITING TO THE CSR. IF A COUNTER IS WRITTEN, YOU DON'T HAVE TO INCREMENT IT.  (pag 22 riscv-public-v2.1.pdf)
          --The problems are only during writing on MCYCLE/H and MINSTRET/H or on any other counters that count csr instructions.
          if (PCER(h)(0) = '1'
              and not(csr_instr_req = '1'
                      and (csr_addr_i = (MCYCLE_addr)
                           or csr_addr_i = MCYCLEH_addr)
                      and (csr_op_i = CSRRWI
                           or csr_op_i = CSRRW
                           or (csr_op_i = CSRRS and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRSI and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRC and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRCI and rs1(instr_word_IE) /= 0)
                           )
                       )
                )
          then  --cycle counter
            if(MCYCLE(h) = x"FFFFFFFF") then
              MCYCLEH(h) <= std_logic_vector(unsigned(MCYCLEH(h))+1);
              MCYCLE(h)  <= x"00000000";
            else
              MCYCLE(h) <= std_logic_vector(unsigned(MCYCLE(h))+1);
            end if;
          end if;

          if (PCER(h)(1) = '1'
              and not(csr_instr_req = '1'
                      and (csr_addr_i = (MINSTRET_addr)
                           or csr_addr_i = MINSTRETH_addr)
                      and (csr_op_i = CSRRWI
                           or csr_op_i = CSRRW
                           or (csr_op_i = CSRRS and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRSI and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRC and rs1(instr_word_IE) /= 0)
                           or (csr_op_i = CSRRCI and rs1(instr_word_IE) /= 0)
                           )
                      )
               )
          then                          --instruction counter
            if(instr_rvalid_i = '1') then
              if (MINSTRET(h) = x"FFFFFFFF") then
                MINSTRETH(h) <= std_logic_vector(unsigned(MINSTRETH(h))+1);
                MINSTRET(h)  <= x"00000000";
              else
                MINSTRET(h) <= std_logic_vector(unsigned(MINSTRET(h))+1);
              end if;
            end if;
          end if;

          if (PCER(h)(2) = '1') then    --load/store access stall
            if (((data_req_o = '1' and data_gnt_i = '0') and data_valid_waiting_counter = '0') or ((not(data_req_o = '1' and data_gnt_i = '0')) and data_valid_waiting_counter = '1')) then
              MHPMCOUNTER3(h) <= std_logic_vector(unsigned(MHPMCOUNTER3(h))+1);
            elsif((data_req_o = '1' and data_gnt_i = '0') and (data_valid_waiting_counter = '1')) then
              MHPMCOUNTER3(h) <= std_logic_vector(unsigned(MHPMCOUNTER3(h))+2);
            end if;
          end if;

          --if(PCER(h)(4)='1') then     --instruction miss 
          --    if ((instr_req_o = '1' and instr_gnt_i = '0') or ( instr_word_flush_bit_IE = '0' and instr_rvalid_i = '0' )) then
          --            MHPMCOUNTER4 <= std_logic_vector(unsigned(MHPMCOUNTER4)+1);
          --    end if;                 
          --end if;     

          if(PCER(h)(5) = '1') then     --load access 
            if (data_req_o = '1' and data_gnt_i = '1' and data_we_o = '0') then
              MHPMCOUNTER6(h) <= std_logic_vector(unsigned(MHPMCOUNTER6(h))+1);
            end if;
          end if;

          if(PCER(h)(6) = '1') then     --store access 
            if (data_req_o = '1' and data_gnt_i = '1' and data_we_o = '1') then
              MHPMCOUNTER7(h) <= std_logic_vector(unsigned(MHPMCOUNTER7(h))+1);
            end if;
          end if;

          if(PCER(h)(7) = '1') then     --jump 
            if (jump_instr = '1') then
              MHPMCOUNTER8(h) <= std_logic_vector(unsigned(MHPMCOUNTER8(h))+1);
            end if;
          end if;

          if(PCER(h)(8) = '1') then     --branch 
            if (branch_instr = '1') then
              MHPMCOUNTER9(h) <= std_logic_vector(unsigned(MHPMCOUNTER9(h))+1);
            end if;
          end if;

          if(PCER(h)(9) = '1') then     --btaken 
            if (branch_instr = '1' and set_branch_condition = '1') then
              MHPMCOUNTER10(h) <= std_logic_vector(unsigned(MHPMCOUNTER10(h))+1);
            end if;
          end if;
        end if;  --debug_req_o='0'
        -- synchronous assignment to MIP bits:
        -- this is Pulpino-specific assignment, i.e. the timer-related IRQ vector value
        MIP(h)(7)  <= '1' when h = 0 and unsigned(irq_id_i) >= 28 and irq_i = '1' else '0'; -- only harc 0 interruptible
        -- this detects the other IRQ vector values in Pulpino
        MIP(h)(11) <= '1' when h = 0 and unsigned(irq_id_i) < 28  and irq_i = '1'  else '0'; -- only harc 0 interruptible
        -- this is the MSIP bit, software interrupt
        MIP(h)(3) <= MIP_3_wire(h);
      end if;  -- reset or ck'event
    end process;

  end generate CSR_updating_logic;
  -- end of replicated logic ------------------------------------------------------------

--here we OR the signals coming from different CS logic replicas
  process(all)
    variable wire1, wire2 : std_logic;
  begin
    wire1 := '0'; wire2 := '0';
    for h in harc_range loop
      wire1 := wire1 or csr_instr_done_replicated(h);
      wire2 := wire2 or csr_access_denied_o_replicated(h);
    end loop;
    csr_instr_done      <= wire1;
    csr_access_denied_o <= wire2;
  end process;

-- this is a mux choosing the csr data output corresponding to the actual harc executed
  csr_rdata_o <= csr_rdata_o_replicated(harc_IE);


-- small fsm using the served_irq signals coming from different PC updating logic replicas
-- to operate on the irq_ack signals  
  irq_ack_manager : process(clk_i, rst_ni)
  variable wire1: std_logic;
  begin
    if rst_ni = '0' then
      irq_ack_o <= '0';
    elsif rising_edge(clk_i) then
      wire1 := '0'; 
      for h in harc_range loop
        wire1 := wire1 or served_irq(h);
      end loop;      
      case irq_ack_o is
        when '0' =>
          if wire1 = '0' then 
            irq_ack_o <= '0';
          else 
            irq_ack_o <= '1';
            irq_id_o <= irq_id_i;
          end if;
        when '1' =>
          irq_ack_o <= '0';
        when others => null;
      end case;
    end if;
  end process irq_ack_manager;

------------------------------------------------------------------------ end of CS Unit ------------
----------------------------------------------------------------------------------------------------  



----------------------------------------------------------------------------------------------------
-- Program Counter Managing Units -- synchronous process, one cycle
-- Note: in the present version, gives priority to branching over trapping, 
-- i.e. branch instructions are not interruptible. This can be changed but may be unsafe.
-- Implements as many hw units as the max number of threads supported
----------------------------------------------------------------------------------------------------

  hardware_context_counter : process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      harc_IF <= THREAD_POOL_SIZE -1;
    elsif rising_edge(clk_i) then
      if instr_gnt_i = '1' then
        harc_IF <= harc_IF - 1 when harc_IF > 0 else THREAD_POOL_SIZE -1;
      end if;
    end if;
  end process hardware_context_counter;

  -- this is the multiplexer on the PC_IF
  pc_IF <= pc(harc_IF);

  -- fixed connections, not replicated 
  boot_pc                      <= boot_addr_i(31 downto 8) & std_logic_vector(to_unsigned(128, 8));
  mepc_incremented_pc(harc_IE) <= std_logic_vector(unsigned(MEPC(harc_IE))+4);
  mepc_interrupt_pc(harc_IE)   <= MEPC(harc_IE);

  ----------------------------------------------------------------------------------------------
  -- this part of logic and registers is replicated as many times as the supported threads:
  pc_update_logic : for h in harc_range generate

    relative_to_PC(h) <= std_logic_vector(to_unsigned(0, 32)) when (absolute_jump = '1')
                         else pc_IE;
    incremented_pc(h) <= std_logic_vector(unsigned(pc(h))+4);
    irq_pending(h)    <= ((MIP(h)(11) or MIP(h)(7) or MIP(h)(3)) and MSTATUS(h)(3));
    -- set conditions and harc_IE are simultaneous:

    set_wfi_condition_replicated(h) <= '1' when set_wfi_condition = '1' and (harc_IE = h) --xxxxxxxx why was it harc_to_csr???
                                       else '0';
    set_branch_condition_replicated(h) <= '1' when set_branch_condition = '1' and (harc_IE = h)
                                          else '0';
    set_except_condition_replicated(h) <= '1' when set_except_condition = '1' and (harc_IE = h)
                                          else '0';
    set_mret_condition_replicated(h) <= '1' when set_mret_condition = '1' and (harc_IE = h)
                                        else '0';
    flush_thread(h) <=
      set_branch_condition_replicated(h)
      or served_irq(h)
      or set_except_condition_replicated(h)
      or set_mret_condition_replicated(h)
      or set_wfi_condition_replicated(h)
      or branch_condition_pending(h)
      or except_condition_pending(h)
      or mret_condition_pending(h)
      or wfi_condition_pending(h);

    -- latch on the branch address, possibly useless but may be needed in future situations

    taken_branch_pc_lat(h) <= std_logic_vector(signed(relative_to_PC(h))+signed(PC_offset(h)))
                              when (set_branch_condition = '1');


    pc_update_enable(h) <= '1' when instr_gnt_i = '1'
                           and (harc_IF = h
                                or set_branch_condition_replicated(h) = '1'
                                or set_except_condition_replicated(h) = '1'
                                or set_mret_condition_replicated(h) = '1'
                                or set_wfi_condition_replicated(h) = '1'
                                or branch_condition_pending(h) = '1'
                                or except_condition_pending(h) = '1'
                                or mret_condition_pending(h) = '1'
                                or wfi_condition_pending(h) = '1')
                           else '0';

    pc_updater : process(clk_i, rst_ni, boot_pc)
    begin
      if rst_ni = '0' then
        pc(h)                       <= boot_pc;  -- better to put boot_pc to ensure clear synthesis
        branch_condition_pending(h) <= '0';
        except_condition_pending(h) <= '0';
        wfi_condition_pending(h)    <= '0';
        mret_condition_pending(h)   <= '0';
        served_except_condition(h)  <= '0';
        served_mret_condition(h)    <= '0';
        served_irq(h)               <= '0';
      elsif rising_edge(clk_i) then
        -- synch.ly updates pc with new value depending on conditions pending 
        -- synch.ly raises "served" signal for the condition that is being served 
        -- synch.ly lowers "served" signal for other conditions
        pc_update(MTVEC(h), MCAUSE(h), sw_mip, instr_gnt_i, set_branch_condition_replicated(h), wfi_condition_pending(h),
                  set_wfi_condition_replicated(h), branch_condition_pending(h), irq_pending(h), set_except_condition_replicated(h),
                  except_condition_pending(h), set_mret_condition_replicated(h), mret_condition_pending(h),
                  pc(h), taken_branch_pc_lat(h), incremented_pc(h), mepc_incremented_pc(h),
                  mepc_interrupt_pc(h), boot_pc, pc_update_enable(h), served_except_condition(h),
                  served_mret_condition(h), served_irq(h));
      end if;  --rst , clk
    end process;

    flush_IF(h) <= '1' when flush_thread(h) = '1' and harc_IF = h else
                   '0';
    flush_ID(h) <= '1' when flush_thread(h) = '1' and harc_ID = h else
                   '0';

  end generate pc_update_logic;
  -- end of replicated logic --   
  --------------------------------------------------------------------------------------------

  -- Flush signals for specific pipeline stages --

  IF_stage_flush_or_gates : process(all)
    variable wire1, wire2, wire3 : std_logic;
  begin
    wire1 := '0';
    for h in harc_range loop
      wire1 := wire1 or flush_IF(h);
    end loop;
    flush_instruction_IF <= wire1;
  end process;

  -- Since we cannot safely flush a flying access in the program memory,
  -- we need to remember if the fetched instruction is to be flushed in ID
  flush_delayer : process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      flush_instruction_previous_IF <= '0';
    elsif rising_edge(clk_i) then
      if instr_gnt_i = '1' then
        flush_instruction_previous_IF <= flush_instruction_IF;
      end if;
    end if;
  end process;

  -- the flush signal in ID stages takes into account also a possibly
  -- erroneously fetched instruction coming from previous fetch
  ID_stage_flush_or_gates : process(all)
    variable wire1, wire2, wire3 : std_logic;
  begin
    wire1 := '0';
    for h in harc_range loop
      wire1 := wire1 or flush_ID(h);
    end loop;
    flush_instruction_ID <= wire1 or flush_instruction_previous_IF;
  end process;
--------------------------------------------------------------------- end of PC Managing Units ---
--------------------------------------------------------------------------------------------------  
  --DEBUG_UNIT--
  --There are two processes, one handle the minterface between the external and the core, and one memorize the debug state.

  DBU_interface_handler : process(clk_i, rst_ni)
  begin
    if(rst_ni = '0') then
      debug_rvalid_o  <= '0';
      debug_rdata_o   <= (others => 'Z');
      dbg_halt_req    <= '0';
      dbg_sse         <= '0';
      dbg_ssh         <= '1';
      dbg_resume_core <= '0';
    elsif rising_edge(clk_i) then
      dbg_ssh <= '1';
      if(debug_req_i = '1') then
        debug_rvalid_o <= '1';
        if(debug_we_i = '0') then       --read access
          case debug_addr_i(13 downto 8) is
            when "000000" =>            --debug register always accessible
              case debug_addr_i(6 downto 2) is
                when "00000" =>
                  debug_rdata_o <= (0      => dbg_halted_o,
                                    16     => dbg_sse,
                                    others => '0');
                when "00001" =>
                  debug_rdata_o <= (0 => dbg_ssh, others => '0');
                when others =>
                  null;
              end case;
            when "100000" =>  --debug regster accessible only when core is halted, that's why there is a condition on dbg_halted_o
              if dbg_halted_o = '1' then
                case debug_addr_i(2) is
                  when '0' =>           --previous pc 
                    debug_rdata_o <= pc_ie;
                  when '1' =>
                    if served_irq(harc_IE) = '1' then  --next program counter, there rows came form the PC handler, but the destination is debug bus and not PC
                      debug_rdata_o <= MTVEC(harc_IE);
                    elsif (not (set_branch_condition = '1' or branch_condition_pending(harc_IE) = '1')
                           and not (((MIP(harc_IE)(11) or MIP(harc_IE)(7) or MIP(harc_IE)(3)) and MSTATUS(harc_IE)(3)) = '1')
                           and not ((set_except_condition or except_condition_pending(harc_IE)) = '1')
                           and not ((set_mret_condition or mret_condition_pending(harc_IE)) = '1'))
                    then
                      debug_rdata_o <= incremented_pc(harc_IE);
                    elsif set_branch_condition = '1' or branch_condition_pending(harc_IE) = '1' then
                      debug_rdata_o <= taken_branch_pc_lat(harc_IE);
                    elsif irq_pending(harc_IE) = '1' then
                      debug_rdata_o <= MTVEC(harc_IE);
                    elsif (set_except_condition or except_condition_pending(harc_IE)) = '1' then
                      debug_rdata_o <= MTVEC(harc_IE);
                    elsif (set_mret_condition or mret_condition_pending(harc_IE)) = '1' and MCAUSE(harc_IE)(31) = '0' then
                      debug_rdata_o <= mepc_incremented_pc(harc_IE);
                    elsif (set_mret_condition or mret_condition_pending(harc_IE)) = '1' and MCAUSE(harc_IE)(31) = '1' then
                      debug_rdata_o <= mepc_interrupt_pc(harc_IE);
                    end if;
                  when others =>
                    null;
                end case;
              end if;
            when "000100" =>            --GPR
              debug_rdata_o <= regfile(harc_IE)(to_integer(unsigned(debug_addr_i(6 downto 2))));
            when others =>
              null;
          end case;
        else                            --WRITE ACCESS
          debug_rvalid_o <= '0';
          case debug_addr_i(13 downto 8) is
            when "000000" =>            --debug register always accessible
              case debug_addr_i(6 downto 2) is
                when "00000" =>
                  if (debug_wdata_i(16) = '1') then
                    if dbg_halted_o = '0' then
                      dbg_halt_req <= '1';
                    else
                      dbg_halt_req <= '0';
                    end if;
                  else
                    if dbg_halted_o = '1' then
                      dbg_resume_core <= '1';
                      dbg_halt_req    <= '0';
                    end if;
                  end if;
                  if(debug_wdata_i(0) = '1') then
                    dbg_sse <= '1';
                  else
                    dbg_sse <= '0';
                  end if;
                when "00001" =>
                  if (debug_wdata_i(0) = '0') then
                    if dbg_sse = '1' and dbg_halted_o = '1' then
                      dbg_ssh <= '0';
                    end if;
                  end if;
                when others =>
                  null;
              end case;
            when others =>
              null;
          end case;
        end if;
      end if;
    end if;
  end process;

  DBU_combinatory_access : process (all)
  begin
    if (debug_req_i = '1') then
      debug_gnt_o <= '1';
    else
      debug_gnt_o <= '0';
    end if;
  end process;

  --DEBUG_UNIT_NEXTSTATE
  fsm_Debug_Unit_nextstate : process(all)

  begin
    if rst_ni = '0' then
      nextstate_DBU <= RUNNING;
      dbg_req_o     <= '0';
      dbg_halted_o  <= '0';
    else
      case state_DBU is
        when RUNNING =>
          dbg_halted_o <= '0';
          if ebreak_instr = '1' then
            dbg_req_o <= '0';
            if dbg_sse = '1' then
              nextstate_DBU <= SINGLE_STEP;
            else
              nextstate_DBU <= HALT;
            end if;
          elsif dbg_halt_req = '1' then
            dbg_req_o <= '1';
            if dbg_sse = '0' then
              nextstate_DBU <= HALT_REQ;
            else
              nextstate_DBU <= SINGLE_STEP_REQ;
            end if;
          else
            dbg_req_o     <= '0';
            nextstate_DBU <= RUNNING;
          end if;
        when HALT_REQ =>
          dbg_halted_o <= '0';
          dbg_req_o    <= '1';
          if dbg_ack_i = '1' then
            nextstate_DBU <= HALT;
          else
            if dbg_resume_core = '1' then
              nextstate_DBU <= RUNNING;
            else
              if dbg_sse = '1' then
                nextstate_DBU <= SINGLE_STEP_REQ;
              else
                nextstate_DBU <= HALT_REQ;
              end if;
            end if;
          end if;  --dbg_ack_i 
        when HALT =>
          dbg_req_o    <= '1';
          dbg_halted_o <= '1';
          if dbg_resume_core = '1' then
            nextstate_DBU <= RUNNING;
          else
            if dbg_sse = '1' then
              nextstate_DBU <= SINGLE_STEP;
            else
              nextstate_DBU <= HALT;
            end if;
          end if;
        when SINGLE_STEP_REQ =>
          dbg_req_o    <= '1';
          dbg_halted_o <= '0';
          if dbg_ack_i = '1' then
            nextstate_DBU <= SINGLE_STEP;
          else
            if dbg_sse = '0' then
              nextstate_DBU <= HALT_REQ;
            else
              nextstate_DBU <= SINGLE_STEP_REQ;
            end if;
          end if;
        when SINGLE_STEP =>
          if dbg_sse = '0' then
            if dbg_resume_core = '1' then
              nextstate_DBU <= RUNNING;
              dbg_req_o     <= '0';
              dbg_halted_o  <= '0';
            else
              dbg_req_o     <= '1';
              dbg_halted_o  <= '1';
              nextstate_DBU <= HALT;
            end if;
          elsif dbg_ssh = '0' then
            dbg_req_o     <= '0';
            dbg_halted_o  <= '0';
            nextstate_DBU <= SINGLE_STEP_REQ;
          else
            dbg_req_o     <= '1';
            dbg_halted_o  <= '1';
            nextstate_DBU <= SINGLE_STEP;
          end if;
        when others =>
          nextstate_DBU <= RUNNING;
          dbg_req_o     <= '0';
          dbg_halted_o  <= '0';
      end case;
    end if;
  end process;


  fsm_DBU_register_state : process(clk_i, rst_ni)
  begin
    if rst_ni = '0' then
      state_DBU <= RUNNING;
    elsif rising_edge(clk_i) then
      state_DBU <= nextstate_DBU;
    end if;
  end process;




end Klessydra_X0;
--------------------------------------------------------------------------------------------------
-- END of Klessydra X0 core architecture --------------------------------------------------------------
--------------------------------------------------------------------------------------------------
