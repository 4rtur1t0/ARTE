function bool=OutOfBounds(q1,q2,q3,q4,q5,q6)

%%ABB1410
maxangle =[deg2rad(-170) deg2rad(170); %Axis 1, minimum, maximum
                 deg2rad(-70) deg2rad(70); %Axis 2
                 deg2rad(-65) deg2rad(70); %Axis 3
                 deg2rad(-150) deg2rad(150); %Axis 4
                 deg2rad(-115) deg2rad(115); %Axis 5
                 deg2rad(-300) deg2rad(300)]; %Axis 6



if (q1<maxangle(1,1)||q1>maxangle(1,2))...
        ||(q2<maxangle(2,1)||q2>maxangle(2,2))...
        ||(q3<maxangle(3,1)||q3>maxangle(3,2))...
        ||(q4<maxangle(4,1)||q4>maxangle(4,2))...
        ||(q5<maxangle(5,1)||q5>maxangle(5,2))...
        ||(q6<maxangle(6,1)||q6>maxangle(6,2))
    
    bool=false;
else
   bool=true; 
end