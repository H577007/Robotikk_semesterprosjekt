            %initialiserer variablar
q1 =-pi/4; q2 = -pi/2.5; q3 = -pi/1.5; q4 = -pi/3.5;
            %setter inn link lengdene (a4 er anslått lengde av
            %pallegafflane)
a1 = 30; a2 = 30; a3 = 10;
            %Legger inn linkane på robotarmen
L(1) = Link([q1 10 0 -pi/2]);
L(2) = Link([q2 0 a1 pi]);
L(3) = Link([q3 0 a2 pi]);
L(4) = Link([q4 0 a3 pi/2]);

            %skriver ut linkane og parameter i Command window
L

            %skriver ut DH parameter i tabell i Command window
four_link=SerialLink(L,'name','My_Robot')

            %forward kinematic av robotarmen dvs. 0T4
            %transformasjonsmatrisa
TE = four_link.fkine([q1 q2 q3 q4])



            %4x5 matriser med vinklane til [q1,q2,q3,q4] for motion
            %planning
a = [-pi/4 -pi/2.5 -pi/1.5 -pi/3.5; -pi/4 -pi/8 -pi/4 -pi/8; -pi/4 -pi/6 -pi/4 -pi/10; -pi/4 -pi/2.5 -pi/1.5 -pi/3.5; pi/4 -pi/2.5 -pi/1.5 -pi/3.5];
b = [-pi/4 -pi/8 -pi/4 -pi/8; -pi/4 -pi/6 -pi/4 -pi/10; -pi/4 -pi/2.5 -pi/1.5 -pi/3.5; pi/4 -pi/2.5 -pi/1.5 -pi/3.5; pi/4 -pi/4 -pi/2 -pi/4];


            %s=5 fordi vi vil køyre gjennom for løkka 5 ganger
s=5;
            %for løkke for å kunne køyre ein samanhengande simulering av
            %ulike bevegelsar
for c = 1:s
            %test_start er variablen for start vinklar til armen,
            %oppdaterast frå a matrisa
    test_start = a(c,:);
            %test_end er variablen for end vinklar til armen, oppdaterast
            %frå b matrisa
    test_end = b(c,:);
            %køyrer simulering av robotarmen med parametera over (dette må
            %leggast inn manuelt i simulink. Dette gjerast ved å skrive
            %sl_jspace i commando window for so å endre roboten til
            %"four_link", q0 til "test_start" og qf til "test_end"
    sim('sl_jspace');

end
            %invers kinematikken for robot armen
qi = four_link.ikine(TE, 'mask', [1 1 1 0 0 1])

            %Jacobian matrise for armen med hensyn på base frame
JA = four_link.jacob0([q1 q2 q3 q4])
            %Jacobian matrise for armen med henyn på end effector fram
JAC = four_link.jacobe([q1 q2 q3 q4])

            %Satt inn maksimal momentet end effektoren skal kunne yte, og
            %finner då moment som påverkar kvart ledd
tau = four_link.jacob0([q1 q2 q3 q4])' * [0 0 6.9 0 0 0]';
            %transformerer matrisa og skriver ut i Command window
tau'