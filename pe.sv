`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// Tensor contraction Processing Element (PE)
// Modes:
//   00 - Streaming
//   01 - Broadcast
//   10 - Trace
//////////////////////////////////////////////////////////////////////////////////

module pe #(
    parameter int LANES        = 4,
    parameter int DW           = 32,
    parameter int PE_ROW_ID    = 0,
    parameter int PE_COL_ID    = 0,
    parameter int MUL_LATENCY  = 3,
    parameter int ADD_LATENCY  = 3
)(
    // Clock / Reset
    input  logic               clk,
    input  logic               rst_n,

    // Mode control
    input  logic [1:0]         Mode,          // 00: streaming, 01: broadcast, 10: trace
    input  logic               Clear_acc,     // 清空累加器
    input  logic               Last_mac,      // 最后一次MAC，准备输出

    // Left interface
    input  logic [LANES*DW-1:0] Left_data_in,
    input  logic                Left_valid_in,
    input  logic [LANES*DW-1:0] Left_result_in,
    input  logic                Left_result_valid_in,

    // Right interface
    output logic [DW-1:0]       Right_data_out,
    output logic                Right_valid_out,
    output logic [LANES*DW-1:0] Right_result_out,
    output logic                Right_result_valid_out,
    output logic [DW-1:0]       Right_bcresult_out,
    output logic                Right_bcresult_valid_out,

    // Top interface
    input  logic [LANES*DW-1:0] Top_data_in,
    input  logic                Top_valid_in
);

    //==========================================================================
    // Registers
    //==========================================================================
    logic [DW-1:0] acc_reg       [LANES];
    logic [DW-1:0] bc_acc_reg;

    logic [1:0]          output_delay_counter;
    logic                output_pending;
    logic [LANES*DW-1:0] output_data;

    //==========================================================================
    // Stage 1: input capture + routing
    //==========================================================================
    logic [LANES*DW-1:0] top_data_s1;
    logic [LANES*DW-1:0] left_data_s1;
    logic                valid_s1;
    logic [1:0]          mode_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            top_data_s1  <= '0;
            left_data_s1 <= '0;
            valid_s1     <= 1'b0;
            mode_s1      <= 2'b00;
        end else begin
            top_data_s1  <= Top_data_in;
            valid_s1     <= Top_valid_in && Left_valid_in;
            mode_s1      <= Mode;

            case (Mode)
                2'b00: begin
                    for (int i = 0; i < LANES; i++) begin
                        left_data_s1[i*DW +: DW] <= Left_data_in[0 +: DW];
                    end
                end
                2'b01: left_data_s1 <= Left_data_in;
                2'b10: begin
                    logic [1:0] sel;
                    sel = (PE_ROW_ID - PE_COL_ID) & 2'b11;
                    for (int i = 0; i < LANES; i++) begin
                        left_data_s1[i*DW +: DW] <= Left_data_in[sel*DW +: DW];
                    end
                end
                default: left_data_s1 <= '0;
            endcase
        end
    end

    //==========================================================================
    // Stage 2: mul inputs
    //==========================================================================
    logic [DW-1:0] mul_a_s2 [LANES];
    logic [DW-1:0] mul_b_s2 [LANES];
    logic          valid_s2;
    logic [1:0]    mode_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s2 <= 1'b0;
            mode_s2  <= 2'b00;
            for (int i = 0; i < LANES; i++) begin
                mul_a_s2[i] <= '0;
                mul_b_s2[i] <= '0;
            end
        end else begin
            valid_s2 <= valid_s1;
            mode_s2  <= mode_s1;
            for (int i = 0; i < LANES; i++) begin
                mul_a_s2[i] <= top_data_s1[i*DW +: DW];
                mul_b_s2[i] <= left_data_s1[i*DW +: DW];
            end
        end
    end

    //==========================================================================
    // Multiplier pipeline
    //==========================================================================
    logic [DW-1:0] mul_result   [LANES];
    logic          valid_mul_out;
    logic [1:0]    mode_mul_out;

    logic          valid_mul_pipe [MUL_LATENCY];
    logic [1:0]    mode_mul_pipe  [MUL_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < MUL_LATENCY; i++) begin
                valid_mul_pipe[i] <= 1'b0;
                mode_mul_pipe[i]  <= 2'b00;
            end
        end else begin
            valid_mul_pipe[0] <= valid_s2;
            mode_mul_pipe[0]  <= mode_s2;
            for (int i = 1; i < MUL_LATENCY; i++) begin
                valid_mul_pipe[i] <= valid_mul_pipe[i-1];
                mode_mul_pipe[i]  <= mode_mul_pipe[i-1];
            end
        end
    end

    assign valid_mul_out = valid_mul_pipe[MUL_LATENCY-1];
    assign mode_mul_out  = mode_mul_pipe[MUL_LATENCY-1];

    generate
        for (genvar i = 0; i < LANES; i++) begin : gen_mul
            fp32_multiplier_pipelined #(.LATENCY(MUL_LATENCY)) u_mul (
                .clk(clk),
                .rst_n(rst_n),
                .a(mul_a_s2[i]),
                .b(mul_b_s2[i]),
                .result(mul_result[i])
            );
        end
    endgenerate

    //==========================================================================
    // Streaming/Trace accumulation (4-lane)
    //==========================================================================
    logic [DW-1:0] st_add_a [LANES];
    logic [DW-1:0] st_add_b [LANES];
    logic          st_add_valid;
    logic [1:0]    st_mode;
    logic [DW-1:0] st_add_result [LANES];
    logic          st_result_valid;
    logic [1:0]    st_result_mode;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_add_valid <= 1'b0;
            st_mode      <= 2'b00;
            for (int i = 0; i < LANES; i++) begin
                st_add_a[i] <= '0;
                st_add_b[i] <= '0;
            end
        end else begin
            st_add_valid <= valid_mul_out && (mode_mul_out == 2'b00 || mode_mul_out == 2'b10);
            st_mode      <= mode_mul_out;
            for (int i = 0; i < LANES; i++) begin
                st_add_a[i] <= acc_reg[i];
                st_add_b[i] <= mul_result[i];
            end
        end
    end

    logic st_valid_pipe [ADD_LATENCY];
    logic [1:0] st_mode_pipe [ADD_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ADD_LATENCY; i++) begin
                st_valid_pipe[i] <= 1'b0;
                st_mode_pipe[i]  <= 2'b00;
            end
        end else begin
            st_valid_pipe[0] <= st_add_valid;
            st_mode_pipe[0]  <= st_mode;
            for (int i = 1; i < ADD_LATENCY; i++) begin
                st_valid_pipe[i] <= st_valid_pipe[i-1];
                st_mode_pipe[i]  <= st_mode_pipe[i-1];
            end
        end
    end

    assign st_result_valid = st_valid_pipe[ADD_LATENCY-1];
    assign st_result_mode  = st_mode_pipe[ADD_LATENCY-1];

    generate
        for (genvar i = 0; i < LANES; i++) begin : gen_st_add
            fp32_adder_pipelined #(.LATENCY(ADD_LATENCY)) u_add (
                .clk(clk),
                .rst_n(rst_n),
                .a(st_add_a[i]),
                .b(st_add_b[i]),
                .result(st_add_result[i])
            );
        end
    endgenerate

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || Clear_acc) begin
            for (int i = 0; i < LANES; i++) acc_reg[i] <= '0;
        end else if (st_result_valid) begin
            case (st_result_mode)
                2'b00: for (int i = 0; i < LANES; i++) acc_reg[i] <= st_add_result[i];
                2'b10: for (int i = 0; i < LANES; i++) if (i == PE_COL_ID[1:0]) acc_reg[i] <= st_add_result[i];
                default: ;
            endcase
        end
    end

    //==========================================================================
    // Broadcast reduction tree + scalar accumulator
    //==========================================================================
    logic [DW-1:0] bc_acc_new;
    logic          bc_acc_update_valid;

    broadcast_reduction_tree #(
        .LANES(LANES),
        .DW(DW),
        .ADD_LATENCY(ADD_LATENCY)
    ) u_bc_reduce (
        .clk(clk),
        .rst_n(rst_n || Clear_acc),
        .prod_in(mul_result),
        .prod_valid_in(valid_mul_out && mode_mul_out == 2'b01),
        .bc_acc_in(bc_acc_reg),
        .bc_acc_out(bc_acc_new),
        .bc_acc_valid_out(bc_acc_update_valid)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || Clear_acc) begin
            bc_acc_reg <= '0;
        end else if (bc_acc_update_valid) begin
            bc_acc_reg <= bc_acc_new;
        end
    end

    //==========================================================================
    // Data forwarding (streaming/trace)
    //==========================================================================
    localparam int TOTAL_LATENCY = 1 + 1 + MUL_LATENCY + 1 + ADD_LATENCY;

    logic [DW-1:0] data_forward_pipe       [TOTAL_LATENCY];
    logic          data_forward_valid_pipe [TOTAL_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < TOTAL_LATENCY; i++) begin
                data_forward_pipe[i]       <= '0;
                data_forward_valid_pipe[i] <= 1'b0;
            end
        end else begin
            if (Mode == 2'b00) begin
                data_forward_pipe[0] <= Left_data_in[0 +: DW];
            end else if (Mode == 2'b10) begin
                logic [1:0] sel;
                sel = (PE_ROW_ID - PE_COL_ID) & 2'b11;
                data_forward_pipe[0] <= Left_data_in[sel*DW +: DW];
            end else begin
                data_forward_pipe[0] <= '0;
            end
            data_forward_valid_pipe[0] <= Left_valid_in && (Mode == 2'b00 || Mode == 2'b10);

            for (int i = 1; i < TOTAL_LATENCY; i++) begin
                data_forward_pipe[i]       <= data_forward_pipe[i-1];
                data_forward_valid_pipe[i] <= data_forward_valid_pipe[i-1];
            end
        end
    end

    assign Right_data_out  = data_forward_pipe[TOTAL_LATENCY-1];
    assign Right_valid_out = data_forward_valid_pipe[TOTAL_LATENCY-1];

    //==========================================================================
    // Result output pipeline (streaming/trace)
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || Clear_acc) begin
            Right_result_out       <= '0;
            Right_result_valid_out <= 1'b0;
            output_delay_counter   <= '0;
            output_pending         <= 1'b0;
            output_data            <= '0;
        end else begin
            Right_result_valid_out <= 1'b0;

            if (Last_mac && !output_pending) begin
                case (Mode)
                    2'b00: output_delay_counter <= PE_COL_ID[1:0];
                    2'b10: output_delay_counter <= (2'd3 - PE_COL_ID[1:0]);
                    default: output_delay_counter <= 2'd0;
                endcase

                if (Mode == 2'b00 || Mode == 2'b10) begin
                    output_pending <= 1'b1;
                    for (int i = 0; i < LANES; i++) output_data[i*DW +: DW] <= acc_reg[i];
                end
            end

            if (output_pending) begin
                if (output_delay_counter != 2'd0) begin
                    output_delay_counter <= output_delay_counter - 2'd1;
                end else if (!Left_result_valid_in) begin
                    Right_result_out       <= output_data;
                    Right_result_valid_out <= 1'b1;
                    output_pending         <= 1'b0;
                end
            end

            if (Left_result_valid_in && (!output_pending || output_delay_counter != 2'd0)) begin
                Right_result_out       <= Left_result_in;
                Right_result_valid_out <= 1'b1;
            end
        end
    end

    //==========================================================================
    // Broadcast output
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || Clear_acc) begin
            Right_bcresult_out       <= '0;
            Right_bcresult_valid_out <= 1'b0;
        end else if (Mode == 2'b01 && Last_mac) begin
            Right_bcresult_out       <= bc_acc_reg;
            Right_bcresult_valid_out <= 1'b1;
        end else begin
            Right_bcresult_valid_out <= 1'b0;
        end
    end

endmodule

//==========================================================================
// Broadcast reduction tree
//==========================================================================
module broadcast_reduction_tree #(
    parameter int LANES = 4,
    parameter int DW = 32,
    parameter int ADD_LATENCY = 3
)(
    input  logic         clk,
    input  logic         rst_n,
    input  logic [DW-1:0] prod_in [LANES],
    input  logic          prod_valid_in,
    input  logic [DW-1:0] bc_acc_in,
    output logic [DW-1:0] bc_acc_out,
    output logic          bc_acc_valid_out
);

    logic [DW-1:0] stage1_sum0, stage1_sum1;
    logic          stage1_valid;
    logic [DW-1:0] stage1_bc_acc;

    fp32_adder_pipelined #(.LATENCY(ADD_LATENCY)) u_add0 (
        .clk(clk), .rst_n(rst_n),
        .a(prod_in[0]), .b(prod_in[1]),
        .result(stage1_sum0)
    );

    fp32_adder_pipelined #(.LATENCY(ADD_LATENCY)) u_add1 (
        .clk(clk), .rst_n(rst_n),
        .a(prod_in[2]), .b(prod_in[3]),
        .result(stage1_sum1)
    );

    logic          valid_pipe1   [ADD_LATENCY];
    logic [DW-1:0] bc_acc_pipe1  [ADD_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ADD_LATENCY; i++) begin
                valid_pipe1[i]  <= 1'b0;
                bc_acc_pipe1[i] <= '0;
            end
        end else begin
            valid_pipe1[0]  <= prod_valid_in;
            bc_acc_pipe1[0] <= bc_acc_in;
            for (int i = 1; i < ADD_LATENCY; i++) begin
                valid_pipe1[i]  <= valid_pipe1[i-1];
                bc_acc_pipe1[i] <= bc_acc_pipe1[i-1];
            end
        end
    end

    assign stage1_valid  = valid_pipe1[ADD_LATENCY-1];
    assign stage1_bc_acc = bc_acc_pipe1[ADD_LATENCY-1];

    logic [DW-1:0] stage2_sum;
    logic          stage2_valid;
    logic [DW-1:0] stage2_bc_acc;

    fp32_adder_pipelined #(.LATENCY(ADD_LATENCY)) u_add2 (
        .clk(clk), .rst_n(rst_n),
        .a(stage1_sum0), .b(stage1_sum1),
        .result(stage2_sum)
    );

    logic          valid_pipe2   [ADD_LATENCY];
    logic [DW-1:0] bc_acc_pipe2  [ADD_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ADD_LATENCY; i++) begin
                valid_pipe2[i]  <= 1'b0;
                bc_acc_pipe2[i] <= '0;
            end
        end else begin
            valid_pipe2[0]  <= stage1_valid;
            bc_acc_pipe2[0] <= stage1_bc_acc;
            for (int i = 1; i < ADD_LATENCY; i++) begin
                valid_pipe2[i]  <= valid_pipe2[i-1];
                bc_acc_pipe2[i] <= bc_acc_pipe2[i-1];
            end
        end
    end

    assign stage2_valid  = valid_pipe2[ADD_LATENCY-1];
    assign stage2_bc_acc = bc_acc_pipe2[ADD_LATENCY-1];

    fp32_adder_pipelined #(.LATENCY(ADD_LATENCY)) u_add3 (
        .clk(clk), .rst_n(rst_n),
        .a(stage2_sum), .b(stage2_bc_acc),
        .result(bc_acc_out)
    );

    logic valid_pipe3 [ADD_LATENCY];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ADD_LATENCY; i++) valid_pipe3[i] <= 1'b0;
        end else begin
            valid_pipe3[0] <= stage2_valid;
            for (int i = 1; i < ADD_LATENCY; i++) begin
                valid_pipe3[i] <= valid_pipe3[i-1];
            end
        end
    end

    assign bc_acc_valid_out = valid_pipe3[ADD_LATENCY-1];

endmodule

//==========================================================================
// Floating point placeholders
//==========================================================================
module fp32_multiplier_pipelined #(parameter int LATENCY = 3)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result
);
    logic [31:0] pipe [LATENCY];
    always_ff @(posedge clk) begin
        pipe[0] <= a * b;
        for (int i = 1; i < LATENCY; i++) pipe[i] <= pipe[i-1];
    end
    assign result = pipe[LATENCY-1];
endmodule

module fp32_adder_pipelined #(parameter int LATENCY = 3)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] result
);
    logic [31:0] pipe [LATENCY];
    always_ff @(posedge clk) begin
        pipe[0] <= a + b;
        for (int i = 1; i < LATENCY; i++) pipe[i] <= pipe[i-1];
    end
    assign result = pipe[LATENCY-1];
endmodule

