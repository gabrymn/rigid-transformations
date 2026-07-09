clear; 
clc; 
close all;
figure;
hold on;
grid on;
axis equal;
view(3); 
xlabel('X'); ylabel('Y'); zlabel('Z');

disp('Input dimensioni parallelepipedo');
fprintf("\n");

lx = input('Lato X: ');
ly = input('Lato Y: ');
lz = input('Lato Z: ');

% coefficiente di scala per la visualizzazione degli assi
k = max([lx, ly, lz]) * 2;

vertex = [
    -lx/2, -ly/2,   0;  
     lx/2, -ly/2,   0;  
     lx/2,  ly/2,   0;  
    -lx/2,  ly/2,   0; 
    -lx/2, -ly/2,   lz;  
     lx/2, -ly/2,   lz;  
     lx/2,  ly/2,   lz;  
    -lx/2,  ly/2,   lz  
];

faces = [
    1, 2, 3, 4;  
    5, 6, 7, 8;  
    1, 2, 6, 5;  
    2, 3, 7, 6;  
    3, 4, 8, 7;  
    4, 1, 5, 8   
];

% plot parallelepipedo pose iniziale
plot_parallelepiped(vertex, faces);

% matrice di rototraslazione cumulativa
T_total = eye(4);

% plot sistema di riferimento pose iniziale (world)
plot_rframe(T_total, k);

clc;
status = 1;

fprintf('\n Parallelepipedo [%.2f x %.2f x %.2f]', lx, ly, lz);
fprintf("\n\n")

while status == 1
    
    rframe = input(' Scelta rototraslazione [1] per body, [2] per world: ');
    
    if rframe ~= 1 && rframe ~= 2
        fprintf("\n [Opzione non valida...]\n\n");
        continue
    end
    
    fprintf('\n Inserire i parametri di traslazione:\n\n');
    delta_x = input('  Spostamento lungo X: ');
    delta_y = input('  Spostamento lungo Y: ');
    delta_z = input('  Spostamento lungo Z: ');
    
    fprintf('\n\n Inserire gli angoli di rotazione (in gradi):\n\n');
    alpha_deg = input('  Rotazione attorno a X: ');
    beta_deg  = input('  Rotazione attorno a Y: ');
    gamma_deg = input('  Rotazione attorno a Z: ');
    
    alpha_rad = deg2rad(alpha_deg);
    beta_rad  = deg2rad(beta_deg);
    gamma_rad = deg2rad(gamma_deg);
   
    Rx = [1 0 0;
        0 cos(alpha_rad) -sin(alpha_rad);
        0 sin(alpha_rad)  cos(alpha_rad)];

    Ry = [cos(beta_rad) 0 sin(beta_rad);
        0 1 0;
        -sin(beta_rad) 0 cos(beta_rad)];

    Rz = [cos(gamma_rad) -sin(gamma_rad) 0;
        sin(gamma_rad)  cos(gamma_rad) 0;
        0 0 1];
    
    % matrice rotazione
    R_step = Rz * Ry * Rx;
    
    % matrice traslazione
    t_step = [delta_x; delta_y; delta_z];
    
    % matrice omogena di rototraslazione
    T_step = [R_step t_step;
         0 0 0 1];

    
    if rframe == 1
        T_total = T_total * T_step;
        fprintf("\n [rototraslazione rispetto a body...]");

    else
        T_total = T_step * T_total;
        fprintf("\n [rototraslazione rispetto a world...]");
    
    end
 
   
    % calcolo vertici trasformati
    vertex_hom = [vertex, ones(size(vertex, 1), 1)]';
    vertex_transformed_hom = T_total * vertex_hom;
    vertex_transformed = vertex_transformed_hom(1:3, :)';

    % plot parallelepipedo pose corrente
    plot_parallelepiped(vertex_transformed, faces);

    % plot sistema di riferimento pose corrente
    plot_rframe(T_total, k);


    fprintf("\n\n");
    status = input(' [1] per continuare, [0] per uscire: ');
    
    fprintf("\n\n");
    
end


% separazione rotazione traslazione da matrice finale T

R_final = T_total(1:3, 1:3);
t_final = T_total(1:3, 4);


% calcolo DoF convenzione eulero1 (ZYZ)

phi_rad = atan2(R_final(2,3), R_final(1,3));
theta_rad = atan2(sqrt(R_final(1, 3).^2 + R_final(2, 3).^2), (R_final(3, 3)));
psi_rad = atan2(R_final(3,2), -R_final(3,1));

phi_deg = rad2deg(phi_rad); 
theta_deg = rad2deg(theta_rad);
psi_deg = rad2deg(psi_rad);


% calcolo DoF convenzione eulero3 (XYZ)

roll_rad = atan2(R_final(2,1), R_final(1,1));
pitch_rad  = atan2(-R_final(3,1), sqrt(R_final(3,2)^2 + R_final(3,3)^2));
yaw_rad = atan2(R_final(3,2), R_final(3,3));

roll_deg = rad2deg(roll_rad);
pitch_deg = rad2deg(pitch_rad);
yaw_deg = rad2deg(yaw_rad);


% Stampa DoF

fprintf('\n\n Gradi di libertà pose finale (posizione e orientazione) \n\n')

fprintf('─────────────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────\n');
fprintf('                 │    x    │    y    │    z    │    α    │    β    │    γ\n');
fprintf('─────────────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────\n');
fprintf('  Eulero1 (ZYZ)  │%9.3f│%9.3f│%9.3f│%9.3f│%9.3f│%9.3f\n', ...
    t_final(1), t_final(2), t_final(3), phi_deg, theta_deg, psi_deg);
fprintf('  Eulero3 (XYZ)  │%9.3f│%9.3f│%9.3f│%9.3f│%9.3f│%9.3f\n', ...
    t_final(1), t_final(2), t_final(3), roll_deg, pitch_deg, yaw_deg);
fprintf('─────────────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────\n\n');


% -------------------------------------------------------------------------


function plot_rframe(T, k)
    
    % |e1| = |e2| = |e3| = 1

    O = T(1:3, 4);  % origine 
    e1 = T(1:3, 1); % versore x
    e2 = T(1:3, 2); % versore y
    e3 = T(1:3, 3); % versore z
    
    X_end = O + k * e1; % punto finale asse X
    Y_end = O + k * e2; % punto finale asse Y
    Z_end = O + k * e3; % punto finale asse Z
    
    plot3([O(1) X_end(1)], [O(2) X_end(2)], [O(3) X_end(3)], 'r', 'LineWidth', 3);
    plot3([O(1) Y_end(1)], [O(2) Y_end(2)], [O(3) Y_end(3)], 'g', 'LineWidth', 3);
    plot3([O(1) Z_end(1)], [O(2) Z_end(2)], [O(3) Z_end(3)], 'b', 'LineWidth', 3);
end

function plot_parallelepiped(curr_vertex, faces)

    patch('Vertices', curr_vertex, 'Faces', faces, ...
        'FaceColor', [1, 1, 1], ... 
        'FaceAlpha', 0.6, ...            
        'EdgeColor', 'k', ...            
        'LineWidth', 2);
end


