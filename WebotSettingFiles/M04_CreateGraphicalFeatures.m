% GraphicalFeautres on Ring road #1
fileID = fopen('A01-Graphicalfeature.txt','w');

% Create objects to block drivers' view of the CAVs in the mainstream

DeltaX=6.5;
DeltaZ=2.5;
L_end_initalTrees=100;
NumStep=(Specs_info.L_start-L_end_initalTrees)/DeltaZ;

X0_tree=RR_translation(1,1)+DeltaX/2;
Y0_tree=RR_translation(1,2);
Z0_tree=RR_translation(1,3);

for iStep=1:NumStep
    X_tree=X0_tree+DeltaX;
    Y_tree=Y0_tree;
    Z_tree=Z0_tree+(iStep-1)*DeltaZ;
    LineText= ['SimpleTree {'];
    fprintf(fileID,'%s\r', LineText)
    LineText = [strjoin({' ','translation',...
        num2str(X_tree),num2str(Y_tree),num2str(Z_tree)})] ;
    fprintf(fileID,'%s\r', LineText);

    Tree_name=strrep(strjoin({'"tree(','RR_',num2str(k),'_',...
        num2str(i),'_',num2str(j),')"'}),' ','');
    LineText = [strjoin({' ','name '}),Tree_name];
    fprintf(fileID,'%s\r', LineText);
    %             fprintf(fileID,'%s\r', ' controller ""')
    LineText ='}' ;
    fprintf(fileID,'%s\r', LineText)
end


for k=1:Num_road
    DeltaX=20;

    DeltaZ=250;
    NumStep=(Length_mainroad-LMain_roadGraphicalFeatures)/DeltaZ;

    X0_tree=RR_translation(k,1)-DeltaX/2;
    Y0_tree=RR_translation(k,2);
    Z0_tree=RR_translation(k,3)+LMain_roadGraphicalFeatures;

    X0_forest=RR_translation(k,1)-DeltaX/2;
    Y0_forest=RR_translation(k,2);
    Z0_forest=RR_translation(k,3)+LMain_roadGraphicalFeatures;

    X0_Building=RR_translation(k,1)

    for i=1:NumStep

        for j=1:2
            if i<NumStep/3
                X_tree=X0_tree+DeltaX*(j-1);
                Y_tree=Y0_tree;
                Z_tree=Z0_tree+(i-1)*DeltaZ/2;
                LineText= ['SimpleTree {'];
                fprintf(fileID,'%s\r', LineText)
                LineText = [strjoin({' ','translation',...
                    num2str(X_tree),num2str(Y_tree),num2str(Z_tree)})] ;
                fprintf(fileID,'%s\r', LineText);

                Tree_name=strrep(strjoin({'"tree(','RR_',num2str(k),'_',...
                    num2str(i),'_',num2str(j),')"'}),' ','');
                LineText = [strjoin({' ','name '}),Tree_name];
                fprintf(fileID,'%s\r', LineText);
                %             fprintf(fileID,'%s\r', ' controller ""')
                LineText ='}' ;
                fprintf(fileID,'%s\r', LineText)


                DeltaX1=35;
                DeltaX2=50;
                %forest
                X_forest=X0_forest-DeltaX1/2+DeltaX2*(j-1);
                Y_forest=Y0_forest;
                Z_forest=Z0_forest+((i-1)+(j-1)/2)*DeltaZ;

                LineText= ['Forest {'];

                fprintf(fileID,'%s\r', LineText);
                %             Forest_name=strrep(strjoin({'"tree(','RR_',num2str(k),'_',num2str(i*j),')"'}),' ','');
                %             LineText = [strjoin({' ','name '}),Forest_name];
                %             fprintf(fileID,'%s\r', LineText);
                LineText = [strjoin({' ','translation',...
                    num2str(X_forest),num2str(Y_forest),num2str(Z_forest)})];
                if j==1;
                    fprintf(fileID,'%s\r', '  rotation 0 1 0 -0.262');
                elseif j==2
                    fprintf(fileID,'%s\r', '  rotation 0 1 0 2.88');
                end
                fprintf(fileID,'%s\r', LineText);
                fprintf(fileID,'%s\r', 'shape [')
                fprintf(fileID,'%s\r', ' 0 16')
                fprintf(fileID,'%s\r', ' 7 17')
                fprintf(fileID,'%s\r', ' 12 18')
                fprintf(fileID,'%s\r', ' 6 25')
                fprintf(fileID,'%s\r', ' -7 22')
                fprintf(fileID,'%s\r', ' -15 5')
                fprintf(fileID,'%s\r', ' -20 -7')
                fprintf(fileID,'%s\r', ' -5 -40')
                fprintf(fileID,'%s\r', ' ]')
                fprintf(fileID,'%s\r', ' density 0.4')
                fprintf(fileID,'%s\r', ' type "spruce"')
                fprintf(fileID,'%s\r', ' maxHeight 6')
                fprintf(fileID,'%s\r', ' minHeight 0.5')
                fprintf(fileID,'%s\r', ' maxRadius 1.5')
                fprintf(fileID,'%s\r', ' minRadius 0.5')
                %             fprintf(fileID,'%s\r', ' controller ""')
                fprintf(fileID,'%s\r', ' }')

                DeltaX1=30;
                DeltaX2=50;
            elseif i>=NumStep/3 && i<=2*NumStep/3

                if j==1
                    X_RB=X0_Building-DeltaX1;
                elseif j==2
                    X_RB=X0_Building+DeltaX1;
                end

                %           X_RB=X0_forest-DeltaX1/2+DeltaX2*(j-1);
                Y_RB=Y0_forest;
                Z_RB=Z0_forest+(i-1)*DeltaZ+(j-1)*DeltaZ/2;

                fprintf(fileID,'%s\r', ' ResidentialBuilding {');
                Building_name=strrep(strjoin({'"RB(','RR_',num2str(k),'_',num2str(k),'_',num2str(i*j),')"'}),' ','');
                LineText = [strjoin({' ','name '}),Building_name];
                fprintf(fileID,'%s\r', LineText);
                LineText = [strjoin({' ','translation',...
                    num2str(X_RB),num2str(Y_RB),num2str(Z_RB)})] ;
                fprintf(fileID,'%s\r', LineText);
                fprintf(fileID,'%s\r', '  rotation 0 1 0 1.57');
                fprintf(fileID,'%s\r', '}')

            else
                if j==1
                    X_TTT=X0_Building-DeltaX1;
                elseif j==2
                    X_TTT=X0_Building+DeltaX1;
                end
                Y_TTT=Y0_forest;
                Z_TTT=Z0_forest+(i-1)*DeltaZ+(j-1)*DeltaZ/2;

                fprintf(fileID,'%s\r', 'TheThreeTowers {');
                Tower_name=strrep(strjoin({'"TTT(','RR_',num2str(k),'_',num2str(k),'_',num2str(i*j),')"'}),' ','');
                LineText = [strjoin({' ','name '}),Tower_name];
                fprintf(fileID,'%s\r', LineText);
                LineText = [strjoin({' ','translation',...
                    num2str(X_TTT),num2str(Y_TTT),num2str(Z_TTT)})] ;
                fprintf(fileID,'%s\r', LineText);
                fprintf(fileID,'%s\r', '  rotation 0 1 0 1.5708');
                fprintf(fileID,'%s\r', '}')

            end
        end
    end
end
fclose(fileID);
