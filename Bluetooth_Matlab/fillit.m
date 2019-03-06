%dataStream = []; 
if hasBluetooth == 1
        dataStream = fread(Bt)
 end
MessageFound = 0;

for i = 1:size(dataStream)
    if MessageFound == 1;
          break
     end
    if dataStream(i) == 82 && dataStream(i+1) == 79 && dataStream(i+2) == 66
        MessageFound = 1
        k = i+3;
        for j = 1:6
        Glucosedata(j) = dataStream(k);
        k = k+1;
        end
    end
end
        
    
    Glucosedata= dec2hex(Glucosedata)
    output = hex2dec(strcat(Glucosedata(3,2), Glucosedata(4),Glucosedata(4,2)))
    arrayVals(y) = output
    
    format shortg 
    c = clock; 
    fix(c)
    timeValsBuff = c;
    timeVals = vertcat(timeVals, timeValsBuff)
    y = y+1;
    
    
    
    %Glucosedata = [];
    %dataStream = [];
    z=z+1;