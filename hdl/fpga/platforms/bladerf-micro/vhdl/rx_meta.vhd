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
    use work.bladerf_p.all;
    use work.fifo_readwrite_p.all;

entity rx_meta is    
    port (
        rx_reset               : in    std_logic;
        rx_clock               : in    std_logic;
        rx_enable              : in    std_logic;

        meta_en                : in    std_logic := '0';
        timestamp_reset        : out   std_logic := '1';
        usb_speed              : in    std_logic;
        rx_mux_sel             : in    unsigned;
        rx_overflow_led        : out   std_logic := '1';
        rx_timestamp           : in    unsigned(63 downto 0);

        -- Triggering
        trigger_arm            : in    std_logic;
        trigger_fire           : in    std_logic;
        trigger_master         : in    std_logic;
        trigger_line           : inout std_logic; -- this is not good, should be in/out/oe

        -- Packet to host via FX3
        packet_en              : in    std_logic;
        packet_control         : in    packet_control_t;
        packet_ready           : out   std_logic;

        -- Samples to host via FX3
        sample_fifo_rclock     : in    std_logic;
        sample_fifo_raclr      : in    std_logic;
        sample_fifo_rreq       : in    std_logic;
        sample_fifo_rdata      : out   std_logic_vector(RX_FIFO_T_DEFAULT.rdata'range);
        sample_fifo_rempty     : out   std_logic;
        sample_fifo_rfull      : out   std_logic;
        sample_fifo_rused      : out   std_logic_vector(RX_FIFO_T_DEFAULT.rused'range);

        -- Mini expansion signals
        mini_exp               : in    std_logic_vector(1 downto 0);

        -- Metadata to host via FX3
        meta_fifo_rclock       : in    std_logic;
        meta_fifo_raclr        : in    std_logic;
        meta_fifo_rreq         : in    std_logic;
        meta_fifo_rdata        : out   std_logic_vector(META_FIFO_RX_T_DEFAULT.rdata'range);
        meta_fifo_rempty       : out   std_logic;
        meta_fifo_rfull        : out   std_logic;
        meta_fifo_rused        : out   std_logic_vector(META_FIFO_RX_T_DEFAULT.rused'range)
    );
end entity;

architecture arch of rx_meta is

    signal rx_packet : packet_control_t;
    signal sample_fifo              : rx_fifo_t           := RX_FIFO_T_DEFAULT;
    signal meta_fifo                : meta_fifo_rx_t      := META_FIFO_RX_T_DEFAULT;
begin

    set_timestamp_reset : process(rx_clock, rx_reset)
    begin
        if( rx_reset = '1' ) then
            timestamp_reset <= '1';
        elsif( rising_edge(rx_clock) ) then
            if( meta_en = '1' ) then
                timestamp_reset <= '0';
            else
                timestamp_reset <= '1';
            end if;
        end if;
    end process;


    -- RX sample FIFO
    sample_fifo.aclr   <= sample_fifo_raclr;
    sample_fifo.wclock <= rx_clock;
    U_rx_sample_fifo : entity work.rx_fifo
        generic map (
            LPM_NUMWORDS        => 2**(sample_fifo.wused'length)
        ) port map (
            aclr                => sample_fifo.aclr,

            wrclk               => sample_fifo.wclock,
            wrreq               => sample_fifo.wreq,
            data                => sample_fifo.wdata,
            wrempty             => sample_fifo.wempty,
            wrfull              => sample_fifo.wfull,
            wrusedw             => sample_fifo.wused,

            rdclk               => sample_fifo_rclock,
            rdreq               => sample_fifo_rreq,
            q                   => sample_fifo_rdata,
            rdempty             => sample_fifo_rempty,
            rdfull              => sample_fifo_rfull,
            rdusedw             => sample_fifo_rused
        );


    -- RX meta FIFO
    meta_fifo.aclr   <= meta_fifo_raclr;
    meta_fifo.wclock <= rx_clock;
    U_rx_meta_fifo : entity work.rx_meta_fifo
        generic map (
            LPM_NUMWORDS        => 2**(meta_fifo.wused'length)
        ) port map (
            aclr                => meta_fifo.aclr,

            wrclk               => meta_fifo.wclock,
            wrreq               => meta_fifo.wreq,
            data                => meta_fifo.wdata,
            wrempty             => meta_fifo.wempty,
            wrfull              => meta_fifo.wfull,
            wrusedw             => meta_fifo.wused,

            rdclk               => meta_fifo_rclock,
            rdreq               => meta_fifo_rreq,
            q                   => meta_fifo_rdata,
            rdempty             => meta_fifo_rempty,
            rdfull              => meta_fifo_rfull,
            rdusedw             => meta_fifo_rused
        );

    -- Sample bridge
    U_fifo_writer : entity work.fifo_writer2
        generic map (
            FIFO_USEDW_WIDTH      => sample_fifo.wused'length,
            FIFO_DATA_WIDTH       => sample_fifo.wdata'length,
            META_FIFO_USEDW_WIDTH => meta_fifo.wused'length,
            META_FIFO_DATA_WIDTH  => meta_fifo.wdata'length
        )
        port map (
            clock               =>  rx_clock,
            reset               =>  rx_reset,
            enable              =>  rx_enable,

            usb_speed           =>  usb_speed,
            meta_en             =>  meta_en,
            packet_en           =>  packet_en,
            timestamp           =>  rx_timestamp,
            mini_exp            =>  mini_exp,

            fifo_full           =>  sample_fifo.wfull,
            fifo_usedw          =>  sample_fifo.wused,
            fifo_data           =>  sample_fifo.wdata,
            fifo_write          =>  sample_fifo.wreq,

            packet_control      =>  packet_control,
            packet_ready        =>  packet_ready,

            meta_fifo_full      =>  meta_fifo.wfull,
            meta_fifo_usedw     =>  meta_fifo.wused,
            meta_fifo_data      =>  meta_fifo.wdata,
            meta_fifo_write     =>  meta_fifo.wreq,
            
            overflow_led        =>  rx_overflow_led,
            overflow_count      =>  open,
            overflow_duration   =>  x"ffff"
        );

    -- RX Trigger
    rxtrig : entity work.trigger(async)
        generic map (
            DEFAULT_OUTPUT  => '0'
        )
        port map (
            armed           => trigger_arm,       -- in  sl
            fired           => trigger_fire,      -- in  sl
            master          => trigger_master,    -- in  sl
            trigger_in      => trigger_line,      -- in  sl
            trigger_out     => trigger_line,      -- out sl
            signal_in       => rx_enable,         -- in  sl
            signal_out      => open               -- out sl
        );

end architecture;
