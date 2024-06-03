## bitrate
## -------
## HSE/M
## HSE=MHz, recording during NMS ms
## 
## HSE/M*N
## HSE/m*N/R
## DIV
## -------
## nb of channels
## PDM (double buffer) bufsz for NMS ms = bitrate/8*2*NMS
## fs
## oversampling
## -------
## PDM buf sz
## PCM buf sz
eng2format();
HSEfreq=8M;NMS=1m;
bitrate=[1280,2560,1280,2560,2048,4096,3072,6144]*1k
chan=[1,2,1,2,1,2,1,2]
"--------------------------------------------------------------------------------"
M=[4,4,4,4,5,5,4,5];
N=[96,96,96,96,192,192,192,192];
R=[5,5,5,5,3,3,5,5];
pllinfreq=HSEfreq/M
plloutfreq=pllinfreq*N
plli2sfreq=plloutfreq/R
DIV=plli2sfreq/bitrate
"--------------------------------------------------------------------------------"
sz=bitrate/8*2*NMS
fs=[8k,8k,16k,16k,32k,32k,48k,48k]
ovs=bitrate/fs/chan
"--------------------------------------------------------------------------------"
PDMsz=chan*fs/1000*(NMS*1000)*ovs/16
PCMsz=chan*fs/1000*(NMS*1000)
quit
