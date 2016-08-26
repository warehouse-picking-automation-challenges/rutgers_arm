%[x y z w]
function [quat] = rot2quat(r)

    aa = vrrotmat2vec(r);
    
    if ( abs(aa(4)) < 1e-6 ) 
        quat= NaN;
        return;
    end
   
    aa(4) = aa(4) * 0.5;
    sinAng = sin(aa(4));
    
    quat = [ aa(1)*sinAng aa(2)*sinAng aa(3)*sinAng cos(aa(4))];
   
end