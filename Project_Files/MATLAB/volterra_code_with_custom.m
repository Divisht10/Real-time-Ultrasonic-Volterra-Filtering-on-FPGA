clc;
clear;
close all;
disp('Starting processing...'); 

% --- 1. Load Data ---
adr = 'C:\Users\DELL\Downloads\';
rcvdata = 'chicon1e6vol3pt2T32025-06-15-140802.mat';
load([adr rcvdata]); % Load raw RF data (RcvData, Trans, Receive)
RData = RcvData{1};
RData = double(RcvData{1});

% --- 2. Trim Data ---
[nz, nx, nt] = size(RData);
% Find last depth (z) sample that has any signal
sumOverXT = squeeze(sum(sum(abs(RData),2),3));
lastNonZeroIndex = find(sumOverXT > 0, 1, 'last');
% Trim RData to only include the valid depth range
RData_trimmed = RData(1:lastNonZeroIndex, :, :);

% Load chirp (excitation) signals
y1 = load([adr 'chi7to12cycle16bw100.mat']);
y2 = load([adr 'chi3pt5to6cycle8bw100.mat']);
load([adr rcvdata]); % Re-load metadata (Trans, Receive)
RData = RData_trimmed;
nFrames = size(RData, 3);

% --- 3. Setup Parameters ---
Fs = 4*Trans.frequency; % Sampling frequency
kk = 250/Fs;
[~,ii] = min([Fs-250/ceil(kk), 250/floor(kk) - Fs]);
Fs = ((2-ii)*250/ceil(kk) + (ii-1)*250/floor(kk)); % Recalculate Fs
tFs = 250e6;
ts = 1e-3/Fs; % Sampling time interval
c = 1540; % Speed of sound (m/s)
w2mm = 1e-3*c/Trans.frequency; % Wavelength-to-mm conversion factor
tw1 = y1.TW.Waveform;
tw2 = y2.TW.Waveform;

% Interpolate chirps to match the new sampling frequency Fs
% Define time vectors for clarity
t_orig1 = (1:length(tw1))/tFs;
t_new1  = 1/(Fs*1e6):1/(Fs*1e6):length(tw1)/tFs;

% Use custom_interp1 and TRANSPOSE (.') to make it a row vector
ww1 = custom_interp1(t_orig1, tw1, t_new1).'; 
ww1 = ww1/max(ww1);

% Do the same for the second chirp
t_orig2 = (1:length(tw2))/tFs;
t_new2  = 1/(Fs*1e6):1/(Fs*1e6):length(tw2)/tFs;

ww2 = custom_interp1(t_orig2, tw2, t_new2).'; 
ww2 = ww2/max(ww2);
% Create matched filter templates (time-reversed chirps with zero padding)
FW1 = [zeros(1,length(ww1)) ww1];
FW2 = [zeros(1,length(ww2)) ww2];

% --- 4. Setup Grids ---
L = Receive(1).endSample; % Number of axial samples
start_depth = Receive(1).startDepth*w2mm; end_depth = Receive(1).endDepth*w2mm;
ddz = (end_depth - start_depth)/(L-1);
hh = start_depth:ddz:26; % Axial grid (mm)
ex = Trans.ElementPos(:,1)*w2mm;
ww = ex'; % Lateral grid (mm)
Neighbor_elements = 25; % Aperture size for beamforming
m = 15; % Memory of the Volterra filer

% --- 5. Initialize Volterra & Output Arrays ---
reborn = Volterra_begins(m); % Pre-calculate Volterra indices
env1_all = zeros(length(hh), length(ww), nFrames);
env2_all = zeros(length(hh), length(ww), nFrames);
env22_all = zeros(length(hh), length(ww), nFrames);
env23_all = zeros(length(hh), length(ww), nFrames);

% Define Region of Interest (ROI) for Volterra tuning
roi2_mm_tune = [13.5 14.5 -5 3];
roi2_tune = [find(hh >= roi2_mm_tune(1),1,'first') find(hh >= roi2_mm_tune(2),1,'first') ...
             find(ww >= roi2_mm_tune(3),1,'first') find(ww >= roi2_mm_tune(4),1,'first')];
         
% --- 6. Main Processing Loop ---
for i = 1:1
    fprintf('Processing Frame %d...\n', i);
    
    % --- 6a. Calculate Original Kernels ---
    x = RData(1:L,:,i); % Get current frame (un-normalized)
    fprintf('Calculating original filter kernels on un-normalized data...\n');
    % Matched filter and beamform on the *un-normalized* data
    x2_conv_orig = custom_conv(x', FW2); 
    x2_orig = x2_conv_orig';
    x2_bf_orig = beamform_fpga(x2_orig,ww,hh,ts*c,ex,L,start_depth,Neighbor_elements);
    % Extract the ROI from this un-normalized, beamformed data
    roi_data_orig = x2_bf_orig(roi2_tune(1):roi2_tune(2), roi2_tune(3):roi2_tune(4));
    
    % Tune the filter: find the H kernels that best fit the un-normalized ROI
    H_par = V_tune(roi_data_orig, reborn); 
    fprintf('Original kernels calculated.\n');

    % --- 6b. Kernel Scaling for FPGA ---
    % Find the single largest kernel value (quadratic or cubic)
    H_max = max([max(abs(H_par.h2)), max(abs(H_par.h3))]);
    % Create a new, scaled struct for FPGA-style math
    H_par_scaled = H_par; 
    H_par_scaled.h2 = H_par.h2 / H_max; % Scale kernels to [-1, 1]
    H_par_scaled.h3 = H_par.h3 / H_max; 
    
    % --- 6c. Normalize Input Data ---
    norm_factor = max(abs(x(:))); % Find max value in raw RF data
    x_norm = x / norm_factor;     % Normalize RF data to [-1, 1]
    
    % --- 6d. Process Fundamental Image (x1) ---
    x1_conv = custom_conv(x_norm', FW1); % Matched filtering
    x1 = x1_conv';
    x1 = beamform_fpga(x1,ww,hh,ts*c,ex,L,start_depth,Neighbor_elements); % Beamforming
    
    env1_unscaled = abs(x1); 
    env1 = env1_unscaled * norm_factor; % Re-scale amplitude
    env1 = env1/max(env1(:)); % Normalize for display
    
    % --- 6e. Process Subharmonic Image (x2) ---
    x2_conv = custom_conv(x_norm', FW2); % Matched filtering
    x2 = x2_conv';
    x2_bf_norm = beamform_fpga(x2,ww,hh,ts*c,ex,L,start_depth,Neighbor_elements); % Beamforming
    
    env2_unscaled = abs(x2_bf_norm);
    env2 = env2_unscaled * norm_factor; % Re-scale amplitude
    env2 = env2/max(env2(:)); % Normalize for display
    
    % --- 6f. Process Volterra Images (x22, x23) ---
    % Apply Volterra filter using NORMALIZED data and SCALED kernels
    z2_norm = V_out(x2_bf_norm, H_par_scaled); 
    
    % Get quadratic envelope
    env22_norm = abs(z2_norm(:,:,2)); 
    % Re-scale: (norm_factor^2) for quadratic input, (H_max) for scaled kernel
    env22 = env22_norm * (norm_factor^2) * H_max; 
    env22 = env22/max(env22(:)); % Normalize for display
    
    % Get cubic envelope
    env23_norm = abs(z2_norm(:,:,3)); 
    % Re-scale: (norm_factor^3) for cubic input, (H_max) for scaled kernel
    env23 = env23_norm * (norm_factor^3) * H_max; 
    env23 = env23/max(env23(:));
    
    % Store envelopes
    env1_all(:,:,i) = env1;
    env2_all(:,:,i) = env2;
    env22_all(:,:,i) = env22;
    env23_all(:,:,i) = env23;
end
   save('vol1pt6T1_chi.mat','env1_all','env2_all','env22_all','env23_all','hh','ww','-v7.3');
   
disp('--- PROCESSING COMPLETE ---');
  load('vol1pt6T1_chi.mat');

% --- 7. Plotting and CTR Calculation ---
% Define ROIs for bubble and tissue
bubble_axial_range = [13.5 14.5];
bubble_lateral_range = [-10 10];
tissue_axial_range = [6 7];
tissue_lateral_range = [-10 10];
% Convert ROI limits from mm to indices
bubble_ax_idx = find(hh >= bubble_axial_range(1), 1, 'first') : find(hh <= bubble_axial_range(2), 1, 'last');
bubble_lat_idx = find(ww >= bubble_lateral_range(1), 1, 'first') : find(ww <= bubble_lateral_range(2), 1, 'last');
tissue_ax_idx = find(hh >= tissue_axial_range(1), 1, 'first') : find(hh <= tissue_axial_range(2), 1, 'last');
tissue_lat_idx = find(ww >= tissue_lateral_range(1), 1, 'first') : find(ww <= tissue_lateral_range(2), 1, 'last');

% Calculate Maximum Intensity Projection (MIP) over all frames
mip_images = {
    max(abs(env1_all), [], 3), ...
    max(abs(env2_all), [], 3), ...
    max(abs(env22_all), [], 3), ...
    max(abs(env23_all), [], 3)
};
image_names = {'Fundamental', 'Subharmonic', 'Quadratic Volterra', 'Cubic Volterra'};

% Display MIP images
figure('Name', 'MIP Images', 'Position', [100 100 1600 600]);
for k = 1:4
    mip_img = mip_images{k};
    % Calculate Contrast-to-Tissue Ratio (CTR) for the MIP image
    avg_bubble = mean(mip_img(bubble_ax_idx, bubble_lat_idx), 'all');
    avg_tissue = mean(mip_img(tissue_ax_idx, tissue_lat_idx), 'all');
    CTR_mip = 20*log10(avg_bubble / avg_tissue);
    
    % Display image in dB
    mip_db = 20*log10(mip_img + eps);
    mip_db(mip_db < -80) = -80; % Set dB floor
    subplot(1,4,k);
    imagesc(ww, hh, mip_db);
    axis image;
    set(gca, 'YDir', 'reverse', 'FontSize', 12, 'FontWeight', 'bold');
    colormap(gray(1000));
    colorbar
    xlabel('Lateral (mm)', 'FontSize', 14, 'FontWeight', 'bold');
    ylabel('Axial (mm)', 'FontSize', 14, 'FontWeight', 'bold');
    title(sprintf('%s MIP\nCTR = %.2f dB', image_names{k}, CTR_mip), ...
          'FontSize', 16, 'FontWeight', 'bold');
    hold on;
    % Draw ROI boxes
    rectangle('Position', [bubble_lateral_range(1), bubble_axial_range(1), ...
        diff(bubble_lateral_range), diff(bubble_axial_range)], ...
        'EdgeColor', 'g', 'LineWidth', 2);
    rectangle('Position', [tissue_lateral_range(1), tissue_axial_range(1), ...
        diff(tissue_lateral_range), diff(tissue_axial_range)], ...
        'EdgeColor', 'r', 'LineWidth', 2);
    hold off;
end

% Calculate CTR for each frame
nFrames = size(env1_all, 3);
CTR_values = zeros(nFrames, 4);
for i = 1:nFrames
    envs = {
        abs(env1_all(:,:,i)), ...
        abs(env2_all(:,:,i)), ...
        abs(env22_all(:,:,i)), ...
        abs(env23_all(:,:,i))
    };
    for j = 1:4
        env = envs{j};
        bubble_mean = mean(env(bubble_ax_idx, bubble_lat_idx), 'all');
        tissue_mean = mean(env(tissue_ax_idx, tissue_lat_idx), 'all');
        CTR_values(i,j) = 20*log10(bubble_mean / tissue_mean);
    end
end
% Display mean and std dev of CTR
CTR_mean = mean(CTR_values, 1);
CTR_std  = std(CTR_values, 0, 1);
CTR_table = table(image_names(:), CTR_mean(:), CTR_std(:), ...
    'VariableNames', {'ImageType', 'CTR_Mean_dB', 'CTR_Std_dB'});
fprintf('\n=== CTR Summary for All Frames ===\n');
disp(CTR_table);
  
% =================== USEFUL FUNCTIONS ========================

% Performs delay-and-sum beamforming
function bout = beamform_fpga(rf,xx,zz,ts_c,ex,L,start_depth,M)
num_axial_pixels = length(zz);
num_lateral_pixels = length(xx);
bout = zeros(num_axial_pixels,num_lateral_pixels);
for i = 1:num_lateral_pixels
    for j = 1:num_axial_pixels
        x_pixel =xx(i);
        z_pixel = zz(j);
        % Calculate delays for all elements in the aperture
        [delay_rows, channel_cols] = calculate_pixel_delays(x_pixel,z_pixel,ex,ts_c,L,start_depth,M,i);
        pixel_sum = 0;
        % Sum the delayed RF data
        for n = 1:length(channel_cols)
            sample_to_get = delay_rows(n);
            channel_to_get = channel_cols(n);
            rf_value = rf(sample_to_get,channel_to_get);
            pixel_sum = pixel_sum + rf_value;
        end
        % Store the averaged pixel value
        bout(j,i) = pixel_sum/length(channel_cols);
    end
end
end

% Calculates geometric delays for a single pixel
function [delay_rows,channel_cols] = calculate_pixel_delays(x_pixel,z_pixel,ex,ts_c,L,start_depth,M,center_element_idx)
num_elements =128;
% Define the active aperture (M elements to left/right)
start_element = center_element_idx-M;
if start_element<1
    start_element = 1;
end
end_element = center_element_idx + M;
if end_element>num_elements
    end_element = num_elements;
end
channel_cols = (start_element:end_element)';
num_elements_in_aperture = length(channel_cols);
delay_rows = zeros(num_elements_in_aperture, 1);
% Calculate delay for each element in the aperture
for n = 1:num_elements_in_aperture
    current_ex_pos = ex(channel_cols(n));
    % Round-trip distance
    dd_n = cordic_sqrt((current_ex_pos - x_pixel)^2 + z_pixel^2);
    % Convert distance to sample index 'k'
    k_n = (z_pixel + dd_n - 2*start_depth)/ts_c + 1;
    delay_rows(n) = round(k_n);
end
% Clamp delay indices to be within valid RF data bounds
for n = 1:length(delay_rows)
    current_delay = delay_rows(n);
    if current_delay >L
        delay_rows(n) = L ;
    elseif current_delay<1
        delay_rows(n)=1;
    end
end
end

% Builds the index maps and weights for Volterra kernels
function reborn = Volterra_begins(m)
ord=3;
d{1} = (1:m)';  d{ord} = 0;
l(1) = m;hw{1}=ones(m,1);
dd = nchoosek(1:m,2);
d{2} = [dd; repmat((1:m)',1,2)]; % Indices for 2nd order terms
hw{2} = [2*ones(length(dd),1);ones(m,1)]; % Weights for 2nd order
l(2) = length(d{2});
dd = nchoosek(1:m,3);
ddd = [repelem((1:m)',m-1,2)];
ddx = repmat((1:m)',m,1);
ddx(1:(m+1):end) = [];
d{3} = [dd; [ddd ddx] ;repmat((1:m)',1,3)]; % Indices for 3rd order terms
hw{3} = [6*ones(length(dd),1);3*ones(length(ddd),1);ones(m,1)]; % Weights for 3rd
l(3) = length(d{3});
reborn.d = d;
reborn.hw = hw;
reborn.l = l;
reborn.m = m;
end

% Applies the Volterra filter to the data
function z = V_out(x,fruit) 
m= fruit.m;
h2 = fruit.h2;
h3 = fruit.h3;
hw = fruit.hw;
d = fruit.d;
[L,W] = size(x);
% Pre-multiply kernels by their weights
h2 = h2.*(hw{2});
h3 = h3.*(hw{3});
z = zeros(size(x,1),size(x,2),3); 
% Loop over all pixels
for i = m:L 
    xy = custom_flipud(x(i-(m:-1:1)+1,:)); % Get m-sample memory window
    for j = 1:W 
          yy = xy(:,j);
          
        % Calculate 2nd order terms
        prod2_terms = prod(yy(d{2}),2);
        % Dot product with 2nd order kernel
        z2_val = (prod2_terms)' * h2; 
        z(i,j,2) = z2_val;
        
        % Calculate 3rd order terms
        prod3_terms = prod(yy(d{3}),2);
        % Dot product with 3rd order kernel
        z3_val = (prod3_terms)' * h3; 
        z(i,j,3) = z3_val;
        
    end
end
end

% Tunes the Volterra filter by solving the linear equation G*H = f
function fruit = V_tune(xy,reborn)
d = reborn.d; 
hw = reborn.hw;
l = reborn.l;
m = reborn.m;
sz = size(xy);
count = 0;
xy_flipped = custom_flipud(xy); % Flip to easily grab past samples
% Pre-allocate G matrix
G = zeros((sz(1)-m)*sz(2),sum(l));
% Build G matrix row by row
for j = 1:sz(2) % Loop over lateral
    for i = 1:(sz(1)-m) % Loop over axial
        count = count+1;
        ii = i+(1:m);
        yy = xy_flipped(ii,j); % Get m-sample memory window
        
        % Calculate all polynomial terms
        prod1_terms = (prod(yy(d{1}),2))'; % 1st order
        prod2_terms = (prod(yy(d{2}),2))'; % 2nd order
        prod3_terms = (prod(yy(d{3}),2))'; % 3rd order
        
        % Create the full row for the G matrix
        G_row = [prod1_terms prod2_terms prod3_terms];
        G(count,:) = G_row;
    end
end
% Create the 'f' vector (desired output)
f = xy(1:(sz(1)-m),:);f= f(:);
% Solve for H using pseudo-inverse: H = G \ f
H = pinv(G)*f;
% Unpack the long H vector into h1, h2, h3 kernels
h1 = H(1:l(1));
h2 = H(l(1)+(1:l(2)),1);
h3 = H(l(1)+l(2)+(1:l(3)),1);
% Store kernels in the output struct
fruit.h1 = h1;
fruit.h2 = h2;
fruit.h3 = h3;
fruit.hw = hw;
fruit.m = m;
fruit.d = d;
end

