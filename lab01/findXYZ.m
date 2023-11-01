function M = findXYZ(uvleft, uvright, Pleft, Pright)

    b = [uvright(1)*Pright(3,4) - Pright(1,4);
        uvright(2)*Pright(3,4) - Pright(2,4);
        uvleft(1)*Pleft(3,4) - Pleft(1,4);
        uvleft(2)*Pleft(3,4) - Pleft(2,4)];
    
    A = [Pright(1,1:3) - uvright(1)*Pright(3,1:3)
        Pright(2,1:3) - uvright(2)*Pright(3,1:3)
        Pleft(1,1:3) - uvleft(1)*Pleft(3,1:3)
        Pleft(2,1:3) - uvleft(2)*Pleft(3,1:3)];
    
    M = pinv(A)*b;

end