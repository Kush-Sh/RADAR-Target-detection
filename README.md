# RADAR Target Generation and Detection

Radio Detection and Ranging (RADAR) has proven to be an effective tool for sensing detecting the obstacles in the field. RADAR techniques are used to detect the objects with Doppler frequency shift. Following steps are taken to achieve the goal -

## 1. FMCW Configuration and Moving target generation.

With the help of radar system specification FMCW waveform and moving target is
generated.
```
• Frequency of operation = 77GHz
• Max Range = 200m
• Range Resolution = 1 m
• Max Velocity = 100 m/s

```
Calculations has done for the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW chirp using the requirements above.
```
speed_of_light = 3e8;
range_resolution = 1;
max_range = 200;
range_of_target = 110.00;
velocity_of_target = -20;

B = speed_of_light/(2*range_resolution);
Tchirp = 5.5*2*(max_range/speed_of_light);
slope = B/Tchirp;

```
Signal generation and Moving Target simulation for each time stamp
```
r_t(i) = range_of_target + velocity_of_target*t(i);
td(i) = 2*r_t(i)/speed_of_light;
Tx(i) = cos(2*pi*(fc*t(i) + slope*(t(i)*t(i))/2));
Rx(i) = cos(2*pi*(fc*(t(i) - td(i)) + slope*((t(i) - td(i))*(t(i) - td(i)))/2));

 Mix(i) = Tx(i) * Rx(i);

```
## 2. Range FFT and 2D FFT has been performed on BEAT Signal.

```
signal_fft = fft(Mix,Nr)./Nr;
signal_fft = abs(signal_fft);
signal_fft  = signal_fft(1:(Nr/2));

```
Plotting code for 1D FFT
```
figure ('Name','Range from First FFT')
subplot(2,1,1)

plot(signal_fft)
axis ([0 200 0 1]);

```

<img src="https://github.com/Kush-Sh/RADAR-Target-detection/blob/main/Images/1D%20FFT.jpg" width="1600" height="793" />

```
Mix=reshape(Mix,[Nr,Nd]);

sig_fft2 = fft2(Mix,Nr,Nd);
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

```
Plotting code for 2D FFT
```
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

```
<img src="https://github.com/Kush-Sh/RADAR-Target-detection/blob/main/Images/2D%20FFT.jpg" width="1600" height="793" />

## 3. 2D – CFAR Implementation.

Following steps is taken for implementation of CFAR –
1. Defining the Training and Guard cells with offset value.
Tr = 3; Td = 6; Gr = 2; Gd = 4;
offset = 7;
2. Creating the loop for Sliding window (for each 128 Chirps and 512 Samples in RDM) –
Two FOR loops are created, one for samples and the other loop is created for Chirps
with removing the margin of Training and Guard cells.
3. In this step, locating the training cells and summing up all the signal levels in all
training cells and all the values mentioned in RDM function is converted back to the
normal values.
4. Averaging the sum value of all the training cell. After calculating the threshold value,
it will be converted back to logarithmic value and the offset value is added to this
logarithmic value.
5. Comparing each value of RDM (CUT) with the threshold value. If the value is greater
than threshold value its new value is assigned to 1 else all values assigned to zero.
6. Last step is assigned to edge values to 0 and the target will be detected at 110m as
defined in first task.

```
for i = 1:((Nr/2) - (2*Tr+2*Gr+1))
    for j = 1: (Nd - (2*Td+2*Gd+1))
        noise_level = zeros(1,1);
        threshold = zeros(1,1); 
        for k = j:(j + (2*Td + 2*Gd))
            for l = i:(i + (2*Tr + 2*Gr))
                noise_level = noise_level + (10^(RDM(l,k)/10));               
            end
        end
        threshold = noise_level/((2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1));
        threshold = offset + (10*log10(threshold));
        
        if RDM(i,k) > threshold
            RDM(i,k) = 1;
        else
            RDM(i,k) = 0;
        end
        
    end
end

```
<img src="https://github.com/Kush-Sh/RADAR-Target-detection/blob/main/Images/last%20step.jpg" width="1575" height="774" />

```

for p = 1:512
    for q = 1:128
        if ((RDM(p,q) ~= 1) & (RDM(p,q) ~= 0))
            RDM(p, q) = 0;
        end
    end
end

```
<img src="https://github.com/Kush-Sh/RADAR-Target-detection/blob/main/Images/2D-CFAR.jpg" width="1600" height="793" />

## Installation and Software for project

MATLAB file is attached for the process mentioned above. Just open the radar_target_detection.m with MATLAB software for editing and simulation purpose.
