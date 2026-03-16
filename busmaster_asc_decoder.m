function signals = busmaster_asc_decoder(asc_file)
% BUSMASTER_ASC_DECODER  Parse a BusMaster .asc log, decode VECV BMS signals.
%
%   signals = busmaster_asc_decoder('CAN3_S0_1.asc')
%
%   Bypasses canMessageImport/canSignalImport - those functions do not
%   support BusMaster .asc format (only Vector .asc and Kvaser .txt).
%
%   BusMaster .asc line format (HEX mode, from log header):
%     <Time_s>  <Tx/Rx>  <Channel>  <CAN_ID_hex>  <Type>  <DLC>  <B0>..<B7>
%   Example:
%     0.0123  Rx  1  18FF04F4  X  8  00 3E 00 64 10 00 FF FF
%
%   Returns a struct where each field is a struct with .Time and .Data
%   vectors - same format as canSignalImport output.
%
%   Signals decoded (source: VECV_BMS_CANDBC.xml):
%     BMS_PackCurrent, BMS_PackVoltage, BMS_SOC,
%     BMS_MaxCellTemp, BMS_MinCellTemp,
%     BMS_PackMaxCellVoltage, BMS_PackMinCellVoltage,
%     Battery_SOH, BMS_PackState,
%     AverageCellVoltage, AverageCellTemperature, BMS_RealSOC

%% DBC signal table
% Columns: name | CAN_ID | start_bit | bit_len | is_signed | factor | offset
% All signals use Intel (little-endian) byte order.

dbc = {
    'BMS_PackCurrent',         0x18FF04F4,  16, 12, true,  1,     0;
    'BMS_PackVoltage',         0x18FF04F4,  32, 10, false, 1,     0;
    'BMS_SOC',                 0x18FF00F4,  56,  7, false, 1,     0;
    'BMS_MaxCellTemp',         0x18FF03F4,   8,  8, false, 1,   -40;
    'BMS_MinCellTemp',         0x18FF03F4,  16,  8, false, 1,   -40;
    'BMS_PackMaxCellVoltage',  0x18FF02F4,  16, 12, false, 0.001, 0;
    'BMS_PackMinCellVoltage',  0x18FF02F4,  40, 12, false, 0.001, 0;
    'Battery_SOH',             0x18FF01F4,  40,  8, false, 0.5,   0;
    'BMS_PackState',           0x18FF01F4,  12,  4, false, 1,     0;
    'AverageCellVoltage',      0x18FF09F4,   0, 16, false, 0.001, 0;
    'AverageCellTemperature',  0x18FF09F4,  16,  8, false, 1,   -40;
    'BMS_RealSOC',             0x18FF0CF4,  56,  8, false, 1,     0;
};

n_sigs     = size(dbc, 1);
msg_ids    = cell2mat(dbc(:,2));
unique_ids = unique(msg_ids);

%% Open file

fid = fopen(asc_file, 'r');
if fid == -1
    error('busmaster_asc_decoder: cannot open file: %s', asc_file);
end

%% Pre-allocate storage (5 million rows per signal, trimmed at end)

MAX_ROWS = 5000000;
t_store  = zeros(MAX_ROWS, n_sigs, 'single');
v_store  = zeros(MAX_ROWS, n_sigs, 'single');
n_store  = zeros(1, n_sigs);

fprintf('  Parsing %s ...\n', asc_file);
n_frames = 0;

%% Parse line by line

while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end

    % Skip BusMaster header lines
    if isempty(line)
        continue;
    end
    if line(1) == '*'
        continue;
    end

    % Split into tokens
    tokens = strsplit(strtrim(line));
    if numel(tokens) < 7
        continue;
    end

    % Token 1: timestamp in seconds
    t_val = str2double(tokens{1});
    if isnan(t_val)
        continue;
    end

    % Token 3: CAN ID
    % Extended frames have trailing 'x' (e.g. 18FF04F4x) - strip it
    % Standard frames have no suffix (e.g. 334)
    id_hex = strrep(tokens{3}, 'x', '');
    id_hex = strrep(id_hex,   'X', '');
    % Skip if not valid hex (catches header lines that slipped through)
    if ~all(ismember(upper(id_hex), '0123456789ABCDEF'))
        continue;
    end
    can_id = uint32(hex2dec(id_hex));

    % Check if this message ID is needed
    if ~any(unique_ids == can_id)
        continue;
    end

    % Token 6: DLC  (format: Time Ch ID Rx/Tx Type DLC B0..B7)
    dlc = str2double(tokens{6});
    if isnan(dlc) || dlc < 1
        continue;
    end

    % Tokens 7..end: data bytes in hex
    n_avail = numel(tokens) - 6;
    if n_avail < dlc
        continue;
    end

    % Build uint8 payload (8 bytes, zero-padded)
    payload = uint8(zeros(1, 8));
    for b = 1:min(dlc, 8)
        bval = hex2dec(tokens{6 + b});
        payload(b) = uint8(bval);
    end

    % Decode signals belonging to this CAN ID
    sig_rows = find(msg_ids == can_id);
    for ri = 1:numel(sig_rows)
        r         = sig_rows(ri);
        start_bit = dbc{r, 3};
        bit_len   = dbc{r, 4};
        is_signed = dbc{r, 5};
        factor    = dbc{r, 6};
        offset    = dbc{r, 7};

        raw_val = extract_bits_intel(payload, start_bit, bit_len, is_signed);
        phys    = raw_val * factor + offset;

        idx = n_store(r) + 1;
        if idx <= MAX_ROWS
            t_store(idx, r) = single(t_val);
            v_store(idx, r) = single(phys);
            n_store(r)      = idx;
        end
    end

    n_frames = n_frames + 1;
    if mod(n_frames, 200000) == 0
        fprintf('    ... %.0fk frames parsed\n', n_frames / 1000);
    end
end

fclose(fid);
fprintf('  Total frames parsed: %d\n', n_frames);

%% Build output struct

signals = struct();
for i = 1:n_sigs
    name = dbc{i, 1};
    n    = n_store(i);
    if n == 0
        warning('busmaster_asc_decoder: no data for signal %s (CAN ID 0x%08X)', ...
                name, dbc{i,2});
        signals.(name).Time = 0;
        signals.(name).Data = NaN;
    else
        signals.(name).Time = double(t_store(1:n, i));
        signals.(name).Data = double(v_store(1:n, i));
    end
    fprintf('  %-28s %d samples\n', name, n);
end

end % main function


%% -----------------------------------------------------------------------
function val = extract_bits_intel(payload, start_bit, bit_len, is_signed)
% Extract bit_len bits starting at start_bit from an 8-byte Intel payload.
% Intel = little-endian: start_bit is the LSbit position.

% Pack 8 bytes into a uint64
raw = uint64(0);
for b = 0:7
    raw = bitor(raw, bitshift(uint64(payload(b+1)), b*8));
end

% Shift and mask
mask = uint64(bitshift(uint64(1), bit_len) - 1);
val  = double(bitand(bitshift(raw, -int32(start_bit)), mask));

% Sign extension
if is_signed
    sign_bit_val = bitshift(uint64(1), bit_len - 1);
    if val >= double(sign_bit_val)
        val = val - double(bitshift(uint64(1), bit_len));
    end
end

end % extract_bits_intel
