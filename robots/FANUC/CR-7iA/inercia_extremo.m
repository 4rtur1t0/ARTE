function inercia = inercia_extremo(radio,masa,longitud)
% Calcula la inercia respecto al eje de revolución
inercia = masa*(radio^2/4 + longitud^2/3);
end