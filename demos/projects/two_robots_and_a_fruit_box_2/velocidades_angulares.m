function [w]=velocidades_angulares(R)
    j=1;
    m=1;
    b=1;
    
    %Desmontamos la matriz
    while j<100
        r11(j)=R{j}(1,1);
        r12(j)=R{j}(1,2);
        r13(j)=R{j}(1,3);
        
        r21(j)=R{j}(2,1);
        r22(j)=R{j}(2,2);
        r23(j)=R{j}(2,3);
        
        r31(j)=R{j}(3,1);
        r32(j)=R{j}(3,2);
        r33(j)=R{j}(3,3);
        
        j=j+1;
    end
    
    %Derivamos los vectores
    rd11=diff(r11);
    rd12=diff(r12);
    rd13=diff(r13);
    
    rd21=diff(r21);
    rd22=diff(r22);
    rd23=diff(r23);
    
    rd31=diff(r31);
    rd32=diff(r32);
    rd33=diff(r33);
    
    %Motamos la matriz derivada
    while m<99
        dR{m}(1,1)=rd11(m);
        dR{m}(1,2)=rd12(m);
        dR{m}(1,3)=rd13(m);
        
        dR{m}(2,1)=rd21(m);
        dR{m}(2,2)=rd22(m);
        dR{m}(2,3)=rd23(m);
        
        dR{m}(3,1)=rd31(m);
        dR{m}(3,2)=rd32(m);
        dR{m}(3,3)=rd33(m);
        
        m=m+1;
    end
    
    %Multiplicamos dR por R(transpuesta) para sacar w
    while b<99
        w{b}=dR{b}*(R{b}.');
        
        b=b+1;
    end
end