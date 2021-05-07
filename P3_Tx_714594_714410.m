%% Practica 3 
% Alejandro Gudiño Gallegos
% Eduardo Ethandrake Castillo Pulido
close all force;
clear;
clc;

%% Contruir Header
preamble = [1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0]';
SFD = [1 0 1 0 1 0 1 1]';
%DSA = de2bi(uint8('Practica II FASE II: Yo y Tu'),8,'left-msb'); 
%DSA = reshape(DSA',1,numel(DSA))';
MAC_D = 'F80DAC209CEF';
MAC_S = '6C71D9591D43';
DSA = uint8(hexToBinaryVector([MAC_D, MAC_S]))';
load lena512.mat; img = uint8(lena512);
%load momo512.mat; img = uint8(M);
%img = img(248:247+33,245:244+45,1); % Lena eye 33x45 pixels
imshow(img); 
size_img=de2bi(size(img),16,'left-msb');
header= [size_img(1,:) size_img(2,:)]';
dim_r = bi2de(header(1:16)', 'left-msb');
dim_c = bi2de(header(17:32)', 'left-msb');
fprintf("La dimension de la imagen es de %d * %d\n", dim_r, dim_c);
payload = de2bi(img,8,'left-msb'); %usar esta de preferencia
payload = payload'; 
payload = payload(:); 
bits2Tx = [preamble; SFD; DSA; header; payload];

%% Parametros para Transmitir
Fs = 192e3;
Ts = 1/Fs;
beta = 0.2;
B = 20000;
Rb = 2*B/(1+beta);
mp = ceil(Fs/Rb);
Rb = Fs/mp;
Tp = 1/Rb;
B = (Rb*(1+beta)/2);
D = 6;
type = 'srrc';
E = Tp;
[pbase ~] = rcpulse(beta, D, Tp, Ts, type, E);
wvtool(pbase);

%% Usar codigo MLT-3
bits_2Tx = reshape(bits2Tx, 4, [])';
address = bi2de(bits_2Tx, 'left-msb');
address = address + 1;
enc4b5b = [1,1,1,1,0; 0,1,0,0,1; 1,0,1,0,0;
           1,0,1,0,1; 0,1,0,1,0; 0,1,0,1,1;
           0,1,1,1,0; 0,1,1,1,1; 1,0,0,1,0;
           1,0,0,1,1; 1,0,1,1,0; 1,0,1,1,1;
           1,1,0,1,0; 1,1,0,1,1; 1,1,1,0,0;
           1,1,1,0,1];
mlt3 = enc4b5b(address,:);
mlt3 = mlt3';
mlt3_bits = mlt3(:);
nrzi_bits = mlt3_bits;
tmp = 1;
for i=1:numel(nrzi_bits)
   if(nrzi_bits(i) == 1)
      tmp = ~tmp; 
   end
   if(tmp)
       nrzi_bits(i) = -1;
   else
       nrzi_bits(i) = 1;
   end
end
code = zeros(1,(numel(mlt3_bits)-1)*mp+1);
code(1:mp:end) = nrzi_bits;
pulse_train = conv(code, pbase);
num_samp = mp*16;
delay = (D*mp)/2;
t = 0:Ts:Ts*(numel(pulse_train)-1);
figure; stem(t(1:num_samp + delay),pulse_train(1:num_samp + delay));
title("Señal a tranmsitir"); xlabel('Tiempo (s)'); ylabel('Amplitud');
figure; stem(t(1:mp:num_samp), code(1:mp:num_samp));
title("Bits a tranmsitir"); xlabel('Tiempo (s)'); ylabel('Amplitud');
figure; pwelch(pulse_train, [], [], [],Fs, 'power');
tx_t = numel(pulse_train)/(Rb*mp);
fprintf("El tiempo de transmision va a ser de: %.2f segundos\n", tx_t);
En_pt = Ts*sum(pulse_train.*pulse_train);
Pow = sum(pulse_train.*pulse_train)/numel(pulse_train);
fprintf("La energia del tren de pulsos es: %d\n", En_pt);
fprintf("La potencia del tren de pulsos es: %d\n", Pow);

%% Diagrama de ojo de transmision
ed = comm.EyeDiagram('SampleRate', Fs*mp, 'SamplesPerSymbol', mp, 'YLimits', [-2 2]);
delay_1 = round(numel(pbase)/3);
ed(pulse_train(delay_1+1:end-delay_1)');

%% Transmitir señal de audio
soundsc([zeros(1,Fs/2) pulse_train], Fs);