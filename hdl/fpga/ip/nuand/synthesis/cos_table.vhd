library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;

library work;

entity cos_table is
    generic (
        Q               : positive := 12;
        INPUT_LENGTH    : positive := 16;
        OUTPUT_LENGTH   : positive := 16;
        DPHASE_STEPS_90 : positive := 4;
        IS_SINE_TABLE   : boolean  := false
    );
    port(
        reset   :       in std_logic;
        clock   :       in std_logic;

        phase   :       in unsigned(INPUT_LENGTH-1 downto 0);
        phase_valid :   in std_logic;

        y       :       out signed(OUTPUT_LENGTH-1 downto 0);
        y_valid :       out std_logic
    );
end entity;


architecture arch of cos_table is


    type cos_table_t is array(natural range <>) of signed(OUTPUT_LENGTH-1 downto 0);
    
    function generate_cos_table(steps, qscale : positive) return cos_table_t is
        variable table : cos_table_t(0 to steps + 1) := (others => (others =>'0'));
    begin
        for i in table'range loop
            if (IS_SINE_TABLE) then
                table(i) := to_signed(integer(round(real(qscale)*sin(MATH_PI/2.0 * real(real(i)/real(steps))))),table(i)'length);
            else
                table(i) := to_signed(integer(round(real(qscale)*cos(MATH_PI/2.0 * real(real(i)/real(steps))))),table(i)'length);
            end if;
        end loop;
        return table;
    end function;

    constant table : cos_table_t(0 to DPHASE_STEPS_90 +1) := generate_cos_table(DPHASE_STEPS_90, (2**Q)-1); 


begin

    find_y : process(clock,reset)
        variable cur_index : natural;
    begin
        if reset = '1' then
            y <= (others=>'0');
            y_valid <= '0';
        elsif rising_edge(clock) then
            y_valid <= '0';
            if phase_valid = '1' then 
                y_valid <= '1';
                if phase <= DPHASE_STEPS_90 then
                    y <= (table(to_integer(phase)));
                    cur_index := to_integer(phase);
                elsif phase <= (2*DPHASE_STEPS_90) then
                    if (IS_SINE_TABLE) then
                        y <= (table(DPHASE_STEPS_90 - (to_integer(phase) -(DPHASE_STEPS_90))));
                    else
                        y <= -(table(DPHASE_STEPS_90 - (to_integer(phase) -(DPHASE_STEPS_90))));
                    end if;
                    cur_index := DPHASE_STEPS_90 - (to_integer(phase) -(DPHASE_STEPS_90));
                elsif phase <= (3*DPHASE_STEPS_90) then
                    y <= -(table(to_integer(phase)- (2*DPHASE_STEPS_90)));
                    cur_index := to_integer(phase)- (2*DPHASE_STEPS_90);
                else
                    if (IS_SINE_TABLE) then
                        y <= -(table(DPHASE_STEPS_90 - (to_integer(phase)-(3*DPHASE_STEPS_90))));
                    else
                        y <= (table(DPHASE_STEPS_90 - (to_integer(phase)-(3*DPHASE_STEPS_90))));
                    end if;
                    cur_index := DPHASE_STEPS_90 - (to_integer(phase)-(3*DPHASE_STEPS_90));
                end if;
            end if;
        end if;
    end process;

end architecture;