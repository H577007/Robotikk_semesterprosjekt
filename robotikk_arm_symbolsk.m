syms a1 a2 q1 q2 q3 q4
            %legger inn linkane på robot armen
L(1) = Link([q1 0 0 0]);
L(2) = Link([q2 0 a1 0]);
L(3) = Link([q3 0 a2 0]);
L(4) = Link([q4 0 0 0]);
            %skriver ut linkane i Command window
L

            %symbolsk DH-parameter
four_link = SerialLink(L,'name','My_Robot')
            %symbolsk forward kinematic
TE = four_link.fkine([q1 q2 q3 q4])
            %symbolsk Jacobian matrise med hensyn på base frame
JA = four_link.jacob0([q1 q2 q3 q4]);
JA = simplify(JA)

            %symbolsk Jacobian matrise med hensyn på end effector frame
JAC = four_link.jacobe([q1 q2 q3 q4])