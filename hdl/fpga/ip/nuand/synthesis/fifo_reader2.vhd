-- Copyright (c) 2025 MyTechCatalog LLC
-- Copyright (c) 2017 Nuand LLC
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.fifo_readwrite_p.all;
    use work.fx3_gpif_p.all;

entity fifo_reader2 is
    generic (
        FIFO_USEDW_WIDTH      : natural                 := 12;
        FIFO_DATA_WIDTH       : natural                 := 32;
        META_FIFO_USEDW_WIDTH : natural                 := 3;
        META_FIFO_DATA_WIDTH  : natural                 := 128
    );
    port (
        clock               :   in      std_logic;
        reset               :   in      std_logic;
        enable              :   in      std_logic;

        usb_speed           :   in      std_logic;
        meta_en             :   in      std_logic;
        packet_en           :   in      std_logic;
        timestamp           :   in      unsigned(63 downto 0);

        fifo_usedw          :   in      std_logic_vector(FIFO_USEDW_WIDTH-1 downto 0);
        fifo_read           :   buffer  std_logic := '0';
        fifo_empty          :   in      std_logic;
        fifo_data           :   in      std_logic_vector(FIFO_DATA_WIDTH-1 downto 0);

        packet_control      :   out     packet_control_t;
        packet_empty        :   out     std_logic;
        packet_ready        :   in      std_logic;

        meta_fifo_usedw     :   in      std_logic_vector(META_FIFO_USEDW_WIDTH-1 downto 0);
        meta_fifo_read      :   buffer  std_logic := '0';
        meta_fifo_empty     :   in      std_logic;
        meta_fifo_data      :   in      std_logic_vector(META_FIFO_DATA_WIDTH-1 downto 0);

        underflow_led       :   buffer  std_logic;
        underflow_count     :   buffer  unsigned(63 downto 0);
        underflow_duration  :   in      unsigned(15 downto 0)
  );
end entity;

architecture simple of fifo_reader2 is

    constant DMA_BUF_SIZE_SS    : natural   := GPIF_BUF_SIZE_SS;
    constant DMA_BUF_SIZE_HS    : natural   := GPIF_BUF_SIZE_HS;
    constant NUM_STREAMS        : natural   := fifo_data'length/(packet_control.data'length);
    constant MAX_TIMESTAMP      : unsigned(timestamp'high downto timestamp'low) := (others => '1');

    signal   dma_buf_size       : natural range DMA_BUF_SIZE_HS to DMA_BUF_SIZE_SS := DMA_BUF_SIZE_SS;
    signal   underflow_detected : std_logic := '0';

    type meta_state_t is (
        META_LOAD,
        META_WAIT,
        META_DOWNCOUNT
    );

    type meta_fsm_t is record
        state           : meta_state_t;
        dma_downcount   : natural range 0 to DMA_BUF_SIZE_SS;
        meta_pkt_sop    : std_logic;
        meta_pkt_eop    : std_logic;
        skip_padding    : std_logic;
        meta_read       : std_logic;
        meta_cache      : std_logic_vector(META_FIFO_DATA_WIDTH-1 downto 0);
        meta_p_time     : unsigned(63 downto 0);
        meta_p_time_r   : unsigned(63 downto 0);
        meta_time_go    : std_logic;
        meta_fifo_empty : std_logic;
        meta_fifo_data  : std_logic_vector(META_FIFO_DATA_WIDTH-1 downto 0);
    end record;

    constant META_FSM_RESET_VALUE : meta_fsm_t := (
        state           => META_LOAD,
        dma_downcount   => 0,
        meta_pkt_sop    => '0',
        meta_pkt_eop    => '0',
        skip_padding    => '0',
        meta_read       => '0',
        meta_cache      => (others => '0'),
        meta_p_time     => (others => '-'),
        meta_p_time_r   => (others => '-'),
        meta_time_go    => '0',
        meta_fifo_empty => '1',
        meta_fifo_data  => (others => '0')
    );

    signal meta_current : meta_fsm_t := META_FSM_RESET_VALUE;
    signal meta_future  : meta_fsm_t := META_FSM_RESET_VALUE;

    type fifo_state_t is (
        IDLE,
        READ_PACKET
    );


    type fifo_fsm_t is record
        state               : fifo_state_t;
        samples_left        : natural range 0 to NUM_STREAMS;      
        packet_control      : packet_control_t;
        packet_data_cache   : std_logic_vector(31 downto 0);
        fifo_read           : std_logic;
    end record;

    constant FIFO_FSM_RESET_VALUE : fifo_fsm_t := (
        state               => IDLE,
        packet_control      => PACKET_CONTROL_DEFAULT,
        packet_data_cache   => (others => '0'),
        samples_left        => 0,
        fifo_read           => '0'
    );

    signal fifo_current : fifo_fsm_t := FIFO_FSM_RESET_VALUE;
    signal fifo_future  : fifo_fsm_t := FIFO_FSM_RESET_VALUE;
    signal valid_r      : std_logic_vector(1 downto 0);

begin

    -- Determine the DMA buffer size based on USB speed
    calc_buf_size : process( clock, reset )
    begin
        if( reset = '1' ) then
            dma_buf_size <= DMA_BUF_SIZE_SS;
        elsif( rising_edge(clock) ) then
            if( usb_speed = '0' ) then
                dma_buf_size <= DMA_BUF_SIZE_SS;
            else
                dma_buf_size <= DMA_BUF_SIZE_HS;
            end if;
        end if;
    end process;


    -- ------------------------------------------------------------------------
    -- META FIFO FSM
    -- ------------------------------------------------------------------------

    -- Meta FIFO synchronous process
    meta_fsm_sync : process( clock, reset )
    begin
        if( reset = '1' ) then
            meta_current <= META_FSM_RESET_VALUE;
        elsif( rising_edge(clock) ) then
            meta_current <= meta_future;
        end if;
    end process;

    packet_empty <= '1' when ( meta_current.meta_fifo_empty = '1' and meta_current.state /= META_WAIT ) else '0' ;

    -- Meta FIFO combinatorial process
    meta_fsm_comb : process( all )
        constant  META_NOW      : unsigned(63 downto 0) := (others => '1');
        variable  meta_time     : unsigned(63 downto 0);
        variable  packet_len    : integer;
    begin

        meta_future <= meta_current;

        meta_future.meta_read <= '0';
        meta_future.meta_pkt_sop <= '0';
        meta_future.meta_pkt_eop <= '0';
        meta_future.meta_fifo_empty <= meta_fifo_empty;
        meta_future.meta_fifo_data  <= meta_fifo_data;
        meta_time := unsigned(meta_current.meta_fifo_data(95 downto 32)) - 1;
        meta_future.meta_p_time_r <= meta_time;

        case meta_current.state is

            when META_LOAD =>

                meta_future.skip_padding <= '0';
                meta_future.meta_p_time <= meta_current.meta_p_time_r;
                meta_future.meta_cache  <= meta_current.meta_fifo_data;

                if( meta_current.dma_downcount = NUM_STREAMS ) then
                    meta_future.dma_downcount <= 0;
                end if;

                if( meta_current.meta_fifo_empty = '0' and packet_en = '1' and packet_ready = '1' ) then
                    meta_future.meta_read <= '1';
                    meta_future.state     <= META_WAIT;
                    if( packet_en = '1' or (meta_current.meta_p_time_r > timestamp and meta_current.meta_p_time_r /= META_NOW) ) then
                            meta_future.meta_time_go  <= '0';
                        else
                            meta_future.meta_time_go  <= '1';
                    end if;
                else
                    meta_future.meta_time_go  <= '0';
                end if;

            when META_WAIT =>

                if( packet_en = '1' ) then
                    packet_len := to_integer(unsigned(meta_current.meta_cache(15 downto 0)));
                    if (packet_len > 4) then
                        meta_future.dma_downcount <= packet_len;
                    else
                        meta_future.dma_downcount <= packet_len - 1;
                    end if ;                   
                else
                    meta_future.dma_downcount <= dma_buf_size - 4;
                end if;

                if( (timestamp >= meta_current.meta_p_time or meta_current.meta_p_time = MAX_TIMESTAMP)
                        and packet_en = '1' and packet_ready = '1' ) then
                    meta_future.meta_time_go <= '1';
                    meta_future.state        <= META_DOWNCOUNT;
                    meta_future.meta_pkt_sop <= '1';
                else
                    meta_future.meta_time_go <= '0';
                end if;

            when META_DOWNCOUNT =>

                meta_future.meta_time_go  <= '1';
                if( packet_en = '1' ) then                   
                   if( fifo_current.packet_control.data_valid = '1') then
                      meta_future.dma_downcount <= meta_current.dma_downcount - 1;
                   end if;
                   if( meta_current.dma_downcount <= 1 and packet_ready = '1' ) then
                       meta_future.state <= META_LOAD;
                       meta_future.meta_pkt_eop <= '1';
                   end if;
                end if;

                if( meta_current.meta_cache(0) = '1' ) then
                   meta_future.skip_padding <= '1';
                end if;

            when others =>

                meta_future.state <= META_LOAD;

        end case;

        -- Abort?
        if( (enable = '0') or (meta_en = '0') ) then
            meta_future <= META_FSM_RESET_VALUE;
        end if;

        -- Output assignments
        meta_fifo_read <= meta_current.meta_read;

    end process;


    -- ------------------------------------------------------------------------
    -- SAMPLE FIFO FSM
    -- ------------------------------------------------------------------------

    -- Sample FIFO synchronous process
    fifo_fsm_sync : process( clock, reset )
    begin
        if( reset = '1' ) then
            fifo_current <= FIFO_FSM_RESET_VALUE;
            valid_r <= (others => '0');
        elsif( rising_edge(clock) ) then
            fifo_current <= fifo_future;
            valid_r <= fifo_current.packet_control.data_valid & valid_r(valid_r'high downto 1);
        end if;
    end process;

    -- Sample FIFO combinatorial process
    fifo_fsm_comb : process( all )
    begin

        fifo_future <= fifo_current;

        fifo_future.fifo_read <= '0';
        fifo_future.packet_control.pkt_sop <= meta_current.meta_pkt_sop;

        fifo_future.packet_control.data_valid <= '0';
       
        case fifo_current.state is

            when IDLE =>
                if( packet_en = '1' ) then
                    fifo_future.state <= READ_PACKET;
                    fifo_future.samples_left <= NUM_STREAMS - 1;
                end if;

            when READ_PACKET =>
                if( meta_current.meta_pkt_eop = '1' and meta_current.skip_padding = '1') then
                    fifo_future.samples_left <= NUM_STREAMS - 1;
                end if;

                if( fifo_data'high > 31) then
                    if( fifo_current.samples_left = 0) then
                        fifo_future.packet_control.data <= fifo_current.packet_data_cache;
                    elsif( fifo_current.samples_left = 1) then
                        fifo_future.packet_control.data <= fifo_data(31 downto 0);
                        fifo_future.packet_data_cache   <= fifo_data(63 downto 32);
                    end if;
                else
                    fifo_future.packet_control.data <= fifo_data(31 downto 0);
                end if;
                
                fifo_future.packet_control.pkt_eop <= '0';

                if( meta_current.meta_time_go = '1' and meta_current.dma_downcount > 0 and packet_ready = '1' ) then
                    if( meta_current.dma_downcount = 1 ) then
                        if (valid_r /= "11") then
                            fifo_future.packet_control.pkt_eop <= '1';
                        end if ;
                    end if;
                    
                    fifo_future.packet_control.data_valid <= '1';

                    if( fifo_current.samples_left = NUM_STREAMS - 1) then
                        fifo_future.fifo_read    <= '1';
                     end if;

                    if( fifo_current.samples_left = 0 ) then
                        fifo_future.samples_left <= NUM_STREAMS - 1;
                    else
                        fifo_future.samples_left <= fifo_current.samples_left - 1;
                    end if;
                end if;

            when others =>

                fifo_future.state <= FIFO_FSM_RESET_VALUE.state;

        end case;

        -- Abort?
        if( enable = '0' ) then
            fifo_future.fifo_read <= '0';
            fifo_future.state     <= FIFO_FSM_RESET_VALUE.state;
        end if;

        -- Output assignments
        fifo_read   <= fifo_current.fifo_read;
        packet_control <= fifo_current.packet_control;

    end process;


    -- ------------------------------------------------------------------------
    -- UNDERFLOW
    -- ------------------------------------------------------------------------

    -- Underflow detection
    detect_underflows : process( clock, reset )
    begin
        if( reset = '1' ) then
            underflow_detected <= '0';
        elsif( rising_edge( clock ) ) then
            underflow_detected <= '0';
            if( enable = '1' and fifo_empty = '1' and
                (meta_en = '0' or (meta_en = '1' and meta_current.meta_time_go = '1')) ) then
                underflow_detected <= '1';
            end if;
        end if;
    end process;

    -- Count the number of times we underflow, but only if they are discontinuous
    -- meaning we have an underflow condition, a non-underflow condition, then
    -- another underflow condition counts as 2 underflows, but an underflow condition
    -- followed by N underflow conditions counts as a single underflow condition.
    count_underflows : process( clock, reset )
        variable prev_underflow : std_logic := '0';
    begin
        if( reset = '1' ) then
            prev_underflow  := '0';
            underflow_count <= (others =>'0');
        elsif( rising_edge( clock ) ) then
            if( prev_underflow = '0' and underflow_detected = '1' ) then
                underflow_count <= underflow_count + 1;
            end if;
            prev_underflow := underflow_detected;
        end if;
    end process;

    -- Active high assertion for underflow_duration when the underflow
    -- condition has been detected.  The LED will stay asserted
    -- if multiple underflows have occurred
    blink_underflow_led : process( clock, reset )
        variable downcount : natural range 0 to 2**underflow_duration'length-1 := 0;
    begin
        if( reset = '1' ) then
            downcount     := 0;
            underflow_led <= '0';
        elsif( rising_edge(clock) ) then
            -- Default to not being asserted
            underflow_led <= '0';

            -- Countdown so we can see what happened
            if( downcount /= 0 ) then
                downcount     := downcount - 1;
                underflow_led <= '1';
            end if;

            -- Underflow occurred so light it up
            if( underflow_detected = '1' ) then
                downcount := to_integer(underflow_duration);
            end if;
        end if;
    end process;

end architecture;
