library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity axi_bin_comp is
    generic (
        ADDR_WIDTH : integer := 12;
        C_AXIS_TDATA_WIDTH : integer := 32
    );
    port (
        -- AXI slave interface (input to the FIFO)
        s00_axis_aclk : in std_logic;
        s00_axis_aresetn : in std_logic;
        s00_axis_tdata : in std_logic_vector(C_AXIS_TDATA_WIDTH-1 downto 0);
        s00_axis_tstrb : in std_logic_vector((C_AXIS_TDATA_WIDTH/8)-1 downto 0);
        s00_axis_tvalid : in std_logic;
        s00_axis_tready : out std_logic;
        s00_axis_tlast : in std_logic;

        -- AXI master interface (output of the FIFO)
        m00_axis_aclk : in std_logic;
        m00_axis_aresetn : in std_logic;
        m00_axis_tdata : out std_logic_vector(C_AXIS_TDATA_WIDTH-1 downto 0);
        m00_axis_tstrb : out std_logic_vector((C_AXIS_TDATA_WIDTH/8)-1 downto 0);
        m00_axis_tvalid : out std_logic;
        m00_axis_tready : in std_logic;
        m00_axis_tlast : out std_logic
    );
end axi_bin_comp;

 architecture Behavioral of axi_bin_comp is

    signal wr_ptr_reg, wr_ptr_next : unsigned(ADDR_WIDTH downto 0) := (others => '0');
    signal wr_ptr_gray_reg, wr_ptr_gray_next : unsigned(ADDR_WIDTH downto 0) := (others => '0');
    signal wr_addr_reg : unsigned(ADDR_WIDTH downto 0) := (others => '0');

    signal rd_ptr_reg, rd_ptr_next : unsigned(ADDR_WIDTH downto 0) := (others => '0');
    signal rd_ptr_gray_reg, rd_ptr_gray_next : unsigned(ADDR_WIDTH downto 0) := (others => '0');
    signal rd_addr_reg : unsigned(ADDR_WIDTH downto 0) := (others => '0');

    signal wr_ptr_gray_sync1_reg, wr_ptr_gray_sync2_reg : unsigned(ADDR_WIDTH downto 0) := (others => '0');
    signal rd_ptr_gray_sync1_reg, rd_ptr_gray_sync2_reg : unsigned(ADDR_WIDTH downto 0) := (others => '0');

    signal s00_rst_sync1_reg, s00_rst_sync2_reg, s00_rst_sync3_reg : std_logic := '1';
    signal m00_rst_sync1_reg, m00_rst_sync2_reg, m00_rst_sync3_reg : std_logic := '1';

    type mem_type is array(0 to (2**ADDR_WIDTH)-1) of std_logic_vector(C_AXIS_TDATA_WIDTH downto 0);
    signal mem : mem_type;
    signal mem_read_data_reg : std_logic_vector(C_AXIS_TDATA_WIDTH downto 0) := (others => '0');
    signal mem_read_data_valid_reg, mem_read_data_valid_next : std_logic := '0';
    signal mem_write_data : std_logic_vector(C_AXIS_TDATA_WIDTH downto 0);

    signal m00_data_reg : std_logic_vector(C_AXIS_TDATA_WIDTH downto 0) := (others => '0');
    signal m00_axis_tvalid_reg, m00_axis_tvalid_next : std_logic := '0';

    -- Full and empty conditions
    signal full : std_logic;
    signal empty : std_logic;

    -- Control signals
    signal write, read, store_output : std_logic;
    signal intermediary: std_logic_vector(31 downto 0) := (others => '0');
    signal ok_conv: std_logic;

begin
    -- Assign AXI signals
    s00_axis_tready <= not full and not s00_rst_sync3_reg;
    m00_axis_tvalid <= m00_axis_tvalid_reg;
    mem_write_data <= s00_axis_tlast & s00_axis_tdata;
    m00_axis_tdata <= m00_data_reg(C_AXIS_TDATA_WIDTH-1 downto 0);
    m00_axis_tlast <= m00_data_reg(C_AXIS_TDATA_WIDTH);

    -- Full condition: Two MSBs of write and read pointers differ, while lower bits match
    full <= '1' when (wr_ptr_gray_reg(ADDR_WIDTH) /= rd_ptr_gray_sync2_reg(ADDR_WIDTH) and
                      wr_ptr_gray_reg(ADDR_WIDTH-1) /= rd_ptr_gray_sync2_reg(ADDR_WIDTH-1) and
                      wr_ptr_gray_reg(ADDR_WIDTH-2 downto 0) = rd_ptr_gray_sync2_reg(ADDR_WIDTH-2 downto 0)) else '0';

    -- Empty condition: Pointers match exactly
    empty <= '1' when rd_ptr_gray_reg = wr_ptr_gray_sync2_reg else '0';
    
    process(s00_axis_aclk)
    begin
        if rising_edge(s00_axis_aclk) then
            if s00_axis_aresetn = '0' then
                s00_rst_sync1_reg <= '1';
                s00_rst_sync2_reg <= '1';
                s00_rst_sync3_reg <= '1';
            else
                s00_rst_sync1_reg <= '0';
                s00_rst_sync2_reg <= s00_rst_sync1_reg or m00_rst_sync1_reg;
                s00_rst_sync3_reg <= s00_rst_sync2_reg;
            end if;
        end if;
    end process;
    
    process(m00_axis_aclk)
    begin
        if rising_edge(m00_axis_aclk) then
            if m00_axis_aresetn = '0' then
                m00_rst_sync1_reg <= '1';
                m00_rst_sync2_reg <= '1';
                m00_rst_sync3_reg <= '1';
            else
                m00_rst_sync1_reg <= '0';
                m00_rst_sync2_reg <= s00_rst_sync1_reg or m00_rst_sync1_reg;
                m00_rst_sync3_reg <= m00_rst_sync2_reg;
            end if;
        end if;
    end process;

    -- Write logic
    process(s00_axis_tvalid, full, wr_ptr_reg)
    begin
        write <= '0';
        wr_ptr_next <= wr_ptr_reg;
        wr_ptr_gray_next <= wr_ptr_gray_reg;

        if s00_axis_tvalid = '1' and full = '0' then
            write <= '1';
            wr_ptr_next <= wr_ptr_reg + 1;
            wr_ptr_gray_next <= wr_ptr_next xor (wr_ptr_next srl 1);
        end if;
    end process;

    process(s00_axis_aclk)
    begin
        if rising_edge(s00_axis_aclk) then
            if s00_rst_sync3_reg = '1' then
                wr_ptr_reg <= (others => '0');
                wr_ptr_gray_reg <= (others => '0');
            else
                wr_ptr_reg <= wr_ptr_next;
                wr_ptr_gray_reg <= wr_ptr_gray_next;
            end if;

            wr_addr_reg <= wr_ptr_next;

            if write = '1' then
                mem(to_integer(wr_addr_reg(ADDR_WIDTH-1 downto 0))) <= mem_write_data;
            end if;
        end if;
    end process;

    -- Pointer synchronization between domains
    process(s00_axis_aclk)
    begin
        if rising_edge(s00_axis_aclk) then
            if s00_rst_sync3_reg = '1' then
                rd_ptr_gray_sync1_reg <= (others => '0');
                rd_ptr_gray_sync2_reg <= (others => '0');
            else
                rd_ptr_gray_sync1_reg <= rd_ptr_gray_reg;
                rd_ptr_gray_sync2_reg <= rd_ptr_gray_sync1_reg;
            end if;
        end if;
    end process;

    process(m00_axis_aclk)
    begin
        if rising_edge(m00_axis_aclk) then
            if m00_rst_sync3_reg = '1' then
                wr_ptr_gray_sync1_reg <= (others => '0');
                wr_ptr_gray_sync2_reg <= (others => '0');
            else
                wr_ptr_gray_sync1_reg <= wr_ptr_gray_reg;
                wr_ptr_gray_sync2_reg <= wr_ptr_gray_sync1_reg;
            end if;
        end if;
    end process;

    -- Read logic
    process(store_output, mem_read_data_valid_reg, empty, rd_ptr_reg)
    begin
        read <= '0';
        rd_ptr_next <= rd_ptr_reg;
        rd_ptr_gray_next <= rd_ptr_gray_reg;
        mem_read_data_valid_next <= mem_read_data_valid_reg;

        if store_output = '1' or mem_read_data_valid_reg = '0' then
        --output data not valid or currently being transferred
            if empty = '0' then
            --not empty, so read
                read <= '1';
                mem_read_data_valid_next <= '1';
                rd_ptr_next <= rd_ptr_reg + 1;
                rd_ptr_gray_next <= rd_ptr_next xor (rd_ptr_next srl 1);
            else
                mem_read_data_valid_next <= '0';
            end if;
        end if;
    end process;

    process(m00_axis_aclk)
    begin
        if rising_edge(m00_axis_aclk) then
            if m00_rst_sync3_reg = '1' then
                rd_ptr_reg <= (others => '0');
                rd_ptr_gray_reg <= (others => '0');
                mem_read_data_valid_reg <= '0';
            else
                rd_ptr_reg <= rd_ptr_next;
                rd_ptr_gray_reg <= rd_ptr_gray_next;
                mem_read_data_valid_reg <= mem_read_data_valid_next;
            end if;

            rd_addr_reg <= rd_ptr_next;

            if read = '1' then
                mem_read_data_reg <= mem(to_integer(rd_addr_reg(ADDR_WIDTH-1 downto 0)));
                --ok_conv <= '0';
            end if;
        end if;
    end process;
    
    -- My binarization code
        process(mem_read_data_reg)
        begin
            for i in 0 to 3 loop
                if(mem_read_data_reg((i*8)+7 downto i*8) < x"80") then
                   intermediary( (i*8)+7 downto i*8) <= (others =>'0');
                else
                    intermediary((i*8)+7 downto i*8) <= (others =>'1');        
                end if;
            end loop;
            --ok_conv <= '1';
        end process; 

    -- Output register
    process(m00_axis_tready, m00_axis_tvalid_reg, mem_read_data_valid_reg)
    begin
        store_output <= '0';
        m00_axis_tvalid_next <= m00_axis_tvalid_reg;

        if (m00_axis_tready = '1' or m00_axis_tvalid_reg = '0')  then --and ok_conv = '1'
            store_output <= '1';
            m00_axis_tvalid_next <= mem_read_data_valid_reg;
        end if;
    end process;

    process(m00_axis_aclk)
    begin
        if rising_edge(m00_axis_aclk) then
            if m00_rst_sync3_reg = '1' then
                m00_axis_tvalid_reg <= '0';
            else
                m00_axis_tvalid_reg <= m00_axis_tvalid_next;
            end if;

            if store_output = '1' then
                m00_data_reg <= mem_read_data_reg(C_AXIS_TDATA_WIDTH) & intermediary; --???
            end if;
        end if;
    end process;

end Behavioral;