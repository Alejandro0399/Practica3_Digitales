%% Lectura de Audacity
filename = 'Audio_C.wav';
[x, Fs] = audioread(filename);
Rx_signal = x;
energy_threshold = 0.05; %Modificado para lena completa, valor para lena recortada 0.1
start = find(abs(Rx_signal) > energy_threshold,3,'first');
stop = find(abs(Rx_signal) > energy_threshold,1,'last');
Rx_signal = Rx_signal(start:stop);
Rx_signal = Rx_signal';
%figure;stem(Rx_signal(1:16*mp));

%% Desarrollo del Match Filter
Fs = 96e3;
Ts = 1/Fs;
beta = 0.5;
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
% Match
h = fliplr(pbase);
Sig_fil = (1/mp)*conv(h',Rx_signal);
delay = (D*mp)/2;
%figure; stem(Sig_fil(1:16*mp));

%% Comprobar que la señal este bien recibida mediante el diagrama de ojo
ed = comm.EyeDiagram('SampleRate', Fs*mp, 'SamplesPerSymbol', mp);
delay_eye = round(numel(pbase)/3);
ed(Sig_fil(delay_eye+1:end-delay_eye)');
figure;pwelch(Sig_fil, [], [], [],Fs, 'power'),

%% Sincronizador de simbolo

symSync = comm.SymbolSynchronizer('TimingErrorDetector','Early-Late (non-data-aided)', 'SamplesPerSymbol',mp);
rxSym = symSync(Sig_fil(delay+1:end)');
release(symSync); % Liberar el objeto
figure; stem(rxSym(1:16*mp));
scatterplot(rxSym);

eye_diagram = comm.EyeDiagram('SampleRate', Fs*mp, 'SamplesPerSymbol', mp);
delay_eye = round(numel(pbase)/3);
eye_diagram(rxSym);

%% generacion de la señal transmitida para calcular el BER
preamble = [1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0]';
SFD = [1 0 1 0 1 0 1 1]';
MAC_D = 'F80DAC209CEF';
MAC_S = '6C71D9591D43';
DSA = uint8(hexToBinaryVector([MAC_D, MAC_S]))';
load momo512.mat; img = uint8(M); 
size_img=de2bi(size(img),16,'left-msb');
header= [size_img(1,:) size_img(2,:)]'; 
payload = de2bi(img,8,'left-msb'); %usar esta de preferencia
payload = payload'; 
payload = payload(:); 
bits2Tx = [preamble; SFD; DSA; header; payload];

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

%% Muestreo con sincronizador de simbolos
umbral = 0;
received = ones(1,numel(rxSym));
received_1 = [-1 rxSym(1:end-1)'];
received_2 = rxSym(1:end)';
received((received_2 > umbral & received_1 > umbral) | (received_2 < umbral & received_1 < umbral)) = 0;
recovered_signal = received';
recovered_signal = recovered_signal(3:end-4);
recovered_signal(1) = 1;
[number, error] = biterr(recovered_signal, mlt3_bits);
fprintf("BER al comparar con MLT3: %d. En : %d muestras.\n", error, number);
%% Recover the audio
enc5b4b = [0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,0;
           0,0,0,0; 0,0,0,0; 0,0,0,0; 0,0,0,1; 0,1,0,0; 0,1,0,1;
           0,0,0,0; 0,0,0,0; 0,1,1,0; 0,1,1,1; 0,0,0,0; 0,0,0,0;
           1,0,0,0; 1,0,0,1; 0,0,1,0; 0,0,1,1; 1,0,1,0; 1,0,1,1;
           0,0,0,0; 0,0,0,0; 1,1,0,0; 1,1,0,1; 1,1,1,0; 1,1,1,1; 
           0,0,0,0; 0,0,0,0];
M = reshape(recovered_signal,5,[])';
address_2 = bi2de(M, 'left-msb');
address_2 = address_2 + 1;
MLT3_2 = enc5b4b(address_2,:)';
rxData = MLT3_2(:);

%% Sicronizador de Trama
% Creación del objeto
preamble_detect = comm.PreambleDetector(preamble,'Input','Bit');   % Deteccion de preámbulo. El índice indica dónde termina la trama
idx = preamble_detect(rxData(1:128));  % Ventana de 128 bits
% Una vez que encuentra el índice, se descartan los “bits basura”
% Una forma de hacerlo es la siguiente:
rxData= rxData(idx+1-numel(preamble):end); %56 inicia preamble
SFD_N = rxData(57:56+numel(SFD));
HEADER_N = rxData(56+8+96+1:56+8+96+32);
DIM_R = bi2de(HEADER_N(1:16)', 'left-msb');
DIM_C = bi2de(HEADER_N(17:32)', 'left-msb');
rxData = uint8(rxData(1:56+8+96+32+DIM_R*DIM_C*8));
BER = sum(abs(rxData-bits2Tx))/numel(bits2Tx)*100;
fprintf("BER al comparar con los bit a mandar: %.4f\n", BER);

%% Recortar Lena
data = rxData(56+8+96+32+1:end);
data = reshape(data,4,[]);
bytes = bi2de(reshape(data,8,[])','left-msb');
lena_rx = reshape(bytes',DIM_R,DIM_C);
figure; imshow(uint8(lena_rx));