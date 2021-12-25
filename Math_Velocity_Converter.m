function [Velocity_out] = Math_Velocity_Converter(Velocity_in,mode)
% Mode) 0 : km/h -> m/s
%       1 : m/s -> km/h

switch(mode)
    case 0
        Velocity_out = Velocity_in .* 1000 ./ 3600;
        
    case 1
        Velocity_out = Velocity_in .* 3600 ./ 1000;
        
    otherwise
        Velocity_out = Velocity_in;
end
        

end

