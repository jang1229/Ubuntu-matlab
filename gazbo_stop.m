    check(1,2) =0 ;
    check(1,1) = 0;
    data.Data =check;
    %send (anlgePub,data);
    send (mpcPub,data);
    