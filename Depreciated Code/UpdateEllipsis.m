        function UpdateEllipsis(self, q)
            % Creating an array of 4x4 transforms relating to robot links
            linkTransforms = zeros(4,4,(self.ellipsis.n)+1);                
            linkTransforms(:,:,1) = self.ellipsis.base; % Setting first transform as the base transform

            piFlag = 0;

            centre = [0 0 0;
                      -0.3 0 -0.125;
                      -0.3 0 -0.05;
                      0 0 0;
                      0 0 0;
                      0 0 0];
            
            mult =     [0.5 0.5 0.5;
                      0.66 0.66 0.66;
                      0.66 0.66 0.66;
                      0.5 0.5 0.5;
                      0.4 0.4 0.4;
                      0.125 0.125 0.125];
            
            % Getting the link data of the robot links
            links = self.ellipsis.links;

            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = linkTransforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                linkTransforms(:,:,i + 1) = current_transform;
            end

            for i = 1:length(links)

                A = 0.001;
                D = 0.001;

                if(i >= 2)

                    if(links(i-1).alpha == pi/2)
                        piFlag = ~piFlag;
                    end

                    if(piFlag)
                        A = A + links(i).d;
                        D = D + links(i).a;
                    else
                        A = A + links(i).a;
                        D = D + links(i).d;
                    end
                else
                     A = A + links(i).a;
                     D = D + links(i).d;
                end

                if (D > 0.001)
                    radii = [D, 0.2, 0.2];
                else
                     radii = [A, 0.2, 0.2];
                end
                

                [X, Y, Z] = ellipsoid(centre(i,1), centre(i,2), centre(i,3), radii(1), radii(2), radii(3));

                self.ellipsis.points{i+1} = [X(:)*mult(i,1),Y(:)*mult(i,2),Z(:)*mult(i,2)];
                self.ellipsis.faces{i+1} = delaunay(self.ellipsis.points{i+1}); 

            end

            centre = [0,0,0];
            radii = [0.1,0.1,0.1];

            [X, Y, Z] = ellipsoid(centre(1), centre(2), centre(3), radii(1), radii(2), radii(3));

            self.ellipsis.points{1} = [X(:),Y(:),Z(:)];
            self.ellipsis.faces{1} = delaunay(self.ellipsis.points{1}); 

            self.ellipsis.plot3d(self.model.getpos());
        end