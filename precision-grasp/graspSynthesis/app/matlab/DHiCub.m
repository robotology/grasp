function DH=DHiCub(finger,hand)

    if finger==1
        DH{1}.A=0.0;        DH{1}.D=0;       DH{1}.alpha=-pi/2;   DH{1}.offset=0;
        if (hand=='l')
            DH{1}.alpha=pi/2;
        end
        DH{2}.A=0.021;      DH{2}.D=0.0056;  DH{2}.alpha=0;       DH{2}.offset=0;
        if (hand=='l')
           DH{2}.D=-0.0056;
        end
        DH{3}.A=0.026;      DH{3}.D=0;       DH{3}.alpha=0;       DH{3}.offset=0;
        DH{4}.A=0.022;      DH{4}.D=0;       DH{4}.alpha=0;       DH{4}.offset=0;
        DH{5}.A=0.0168;     DH{5}.D=0;       DH{5}.alpha=-pi/2;   DH{5}.offset=0;
    elseif finger==2
        DH{1}.A=0.0148;     DH{1}.D=0;       DH{1}.alpha=pi/2;    DH{1}.offset=0;
        if (hand=='l')
            DH{1}.alpha=-pi/2;
        end
        DH{2}.A=0.0259;     DH{2}.D=0;       DH{2}.alpha=0;       DH{2}.offset=0;
        DH{3}.A=0.022;      DH{3}.D=0;       DH{3}.alpha=0;       DH{3}.offset=0;
        DH{4}.A=0.0168;     DH{4}.D=0;       DH{4}.alpha=-pi/2;   DH{4}.offset=0;
    elseif finger==3
        DH{1}.A=0.0285;     DH{1}.D=0;       DH{1}.alpha=0;       DH{1}.offset=0;
        DH{2}.A=0.0240;     DH{2}.D=0;       DH{2}.alpha=0;       DH{2}.offset=0;
        DH{3}.A=0.0168;     DH{3}.D=0;       DH{3}.alpha=-pi/2;   DH{3}.offset=0;
    end

end