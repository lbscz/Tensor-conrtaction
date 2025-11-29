`timescale 1ns/1ps
`default_nettype none

// Top input bus feeding column shared caches and PEs.
// Streaming mode stores tiles into column RAMs and replays them with column-skewed broadcast.
// Broadcast/trace modes bypass RAM and drive PEs directly.
module top_input_bus #(
    parameter int LANES       = 4,
    parameter int DW          = 32,
    parameter int NUM_ROWS    = 4,
    parameter int NUM_COLS    = 4,
    parameter int K_ADDR_LEN  = 7,
    parameter int LOOP_WIDTH  = 8
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Mode/config
    input  logic [1:0]                    Mode,          // 00: streaming, 01: broadcast, 10: trace
    input  logic [K_ADDR_LEN-1:0]         K_len,
    input  logic [LOOP_WIDTH-1:0]         Loop_num,

    // Top input interface (one vector per column)
    input  logic [NUM_COLS-1:0][LANES*DW-1:0] Top_data_in,
    input  logic                          Top_valid_in,

    // Status back to boundary cache
    output logic                          streaming_compute_done,

    // Outputs to PEs
    output logic [NUM_COLS-1:0][LANES*DW-1:0] PE_top_data,
    output logic [NUM_COLS-1:0]               PE_top_valid
);
    typedef enum logic [1:0] {
        S_IDLE,
        S_LOAD,
        S_STREAM
    } stream_state_e;

    // Column shared RAMs for streaming mode
    logic [LANES*DW-1:0] col_ram   [NUM_COLS][2**K_ADDR_LEN];
    logic [K_ADDR_LEN-1:0]        wr_addr;
    logic [K_ADDR_LEN-1:0]        stream_idx;
    logic [LOOP_WIDTH-1:0]        loop_cnt;
    stream_state_e                stream_state;
    logic                         tile_loaded;

    // Write path for streaming tile load
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr      <= '0;
            tile_loaded  <= 1'b0;
            stream_state <= S_IDLE;
        end else begin
            case (stream_state)
                S_IDLE: begin
                    tile_loaded <= 1'b0;
                    wr_addr     <= '0;
                    if (Mode == 2'b00 && Top_valid_in) begin
                        stream_state <= S_LOAD;
                    end
                end
                S_LOAD: begin
                    if (Top_valid_in) begin
                        for (int c = 0; c < NUM_COLS; c++) begin
                            col_ram[c][wr_addr] <= Top_data_in[c];
                        end
                        if (wr_addr == K_len - 1'b1) begin
                            tile_loaded  <= 1'b1;
                            stream_state <= S_STREAM;
                            wr_addr      <= '0;
                        end else begin
                            wr_addr <= wr_addr + 1'b1;
                        end
                    end
                end
                S_STREAM: begin
                    if (Mode != 2'b00) begin
                        stream_state <= S_IDLE;
                    end else if (streaming_compute_done) begin
                        stream_state <= S_IDLE;
                    end
                end
                default: stream_state <= S_IDLE;
            endcase
        end
    end

    // Streaming replay control
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stream_idx  <= '0;
            loop_cnt    <= '0;
        end else begin
            if (stream_state != S_STREAM || !tile_loaded) begin
                stream_idx <= '0;
                loop_cnt   <= '0;
            end else if (loop_cnt < Loop_num) begin
                stream_idx <= stream_idx + 1'b1;
                if (stream_idx == (K_len + NUM_COLS - 2)) begin
                    stream_idx <= '0;
                    loop_cnt   <= loop_cnt + 1'b1;
                end
            end
        end
    end

    // Compute done flag in streaming mode
    always_comb begin
        streaming_compute_done = 1'b0;
        if (stream_state == S_STREAM && tile_loaded) begin
            if (loop_cnt >= Loop_num) begin
                streaming_compute_done = 1'b1;
            end
        end
    end

    // Output selection per mode
    always_comb begin
        for (int c = 0; c < NUM_COLS; c++) begin
            PE_top_data[c]  = '0;
            PE_top_valid[c] = 1'b0;
        end

        unique case (Mode)
            2'b00: begin // streaming with skewed broadcast from column RAM
                if (stream_state == S_STREAM && tile_loaded && loop_cnt < Loop_num) begin
                    for (int c = 0; c < NUM_COLS; c++) begin
                        if (stream_idx >= c && (stream_idx - c) < K_len) begin
                            PE_top_data[c]  = col_ram[c][stream_idx - c];
                            PE_top_valid[c] = 1'b1;
                        end
                    end
                end
            end
            2'b01, // broadcast
            2'b10: begin // trace
                for (int c = 0; c < NUM_COLS; c++) begin
                    PE_top_data[c]  = Top_data_in[c];
                    PE_top_valid[c] = Top_valid_in;
                end
            end
            default: begin
                // keep zeros
            end
        endcase
    end

endmodule

`default_nettype wire
