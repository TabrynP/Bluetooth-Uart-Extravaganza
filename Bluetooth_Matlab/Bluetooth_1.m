
dataStream = [];
Glucosedata = [];
timeVals = [1111,1,11,11,11,1.1111];
hasBluetooth = 0;
i = 1;
j = 1;
k = 1; 
y = 1;
z = 1; 

MessageFound = 0;

b = instrhwinfo('Bluetooth');
for i = 1:size(b.RemoteNames)
    if b.RemoteNames(i) == "HC-05"
        b.RemoteNames
        Bt = Bluetooth('HC-05', 1);
        hasBluetooth = 1;
    end
end

        fopen(Bt)

%for i = 1:56
 %   if dataStream(i) == "52" && dataStream(i+1) == "4f" && dataStream(i+2) ==Bt =  