function [goalReached, GlobalBest, countFE] = MFPSO_algorithm()


    %% *********************** Function Description *********************
    %{
    This function implements the Moth-Flame Particle Swarm Optimization algorithm.
    Outputs:
        goalReached: Boolean indicating if the algorithm reached the tolerance before max iterations.
        GlobalBest: Struct containing the best solution and its cost.
            GlobalBest.Position: the best solution
            GlobalBest.Cost: the cost of the best found solution
        countFE: Total number of function evaluations.
    %}

    %% *********************************** Citation **********************
    %{
    Citation (Please cite the following paper paper):
        If you use this code in your research, please cite the following paper:
        %% Bibtex:
        @article{reda2025motor,
          title={Motor Speed Control of Four-wheel Differential Drive Robots Using a New Hybrid Moth-flame Particle Swarm Optimization (MFPSO) Algorithm},
          author={Reda, Mohamed and Onsy, Ahmed and Haikal, Amira Y and Ghanbari, Ali},
          journal={Journal of Intelligent \& Robotic Systems},
          volume={111},
          number={1},
          pages={31},
          year={2025},
          publisher={Springer}
        }

        %% APA Style
        Reda, M., Onsy, A., Haikal, A. Y., & Ghanbari, A. (2025). Motor Speed Control of Four-wheel Differential Drive Robots Using a New Hybrid Moth-flame Particle Swarm Optimization (MFPSO) Algorithm. Journal of Intelligent & Robotic Systems, 111(1), 31.

    
        %% Chicago/Turabian Style
        Reda, Mohamed, Ahmed Onsy, Amira Y. Haikal, and Ali Ghanbari. "Motor Speed Control of Four-wheel Differential Drive Robots Using a New Hybrid Moth-flame Particle Swarm Optimization (MFPSO) Algorithm." Journal of Intelligent & Robotic Systems 111, no. 1 (2025): 31.
        
    Code citation:
        If you use this code in your research, please cite the following code as follows:

        Mohamed Reda. (2024). MATLAB implementation of the Moth-Flame Particle Swarm Optimization (MFPSO) Algorithm. Available at: [https://github.com/MohamedRedaMu/MFPSO]

    Contact Information:
        For questions or further information, please contact:
        Author Name:
            Mohamed Reda
    
        Affiliation:
            1- School of Engineering, University of Central Lancashire, Preston, PR1 2HE, UK
            2- Computers and Control Systems Engineering Department, Faculty of Engineering, Mansoura University, Mansoura, 35516, Egypt
    
        Emails:
            mohamed.reda.mu@gmail.com;
            mohamed.reda@mans.edu.eg;
            mramohamed@uclan.ac.uk

    %}

   %% Begin intialiazation
    %% CEC function paramters
    fNo = 3 ; % function number
    nd = 20; % dim size (must be 10 or 20)
    lb = -100 ; % Lower bounds
    ub = 100 ;  % Upper bounds

    %% Cost Function
    CECyear = 2020;
    costFunction = @(sol) cost_cec2020(sol, fNo);

    %% Algorithm paramters
    AlgName = 'MFPSO Algorithm' ;
    c1 = 1.5; % Personal learning coefficient
    c2 = 0.5; % Global learning coefficient
    w = 0.4; % Inertia weight
    aMin = -2 ; % convergence const min (lower bound)
    aMax = -1 ; %convergece cost max (upper bound)
    b =1 ; % constant for defining the shape of the logarithmic spiral
    nPop = 30; % Number of moths

    %% Stopping criteria
    tol = 10^-8;
    if nd == 10
        maxfe = 200000 ; 
    elseif nd == 20 
        maxfe = 500000;
    else
        fprintf('Dimensions must be 10 or 20 \n');
        return
    end

    %% Display iteration prompt
    print_flag = true;

    %%  Global variable to count number of function evaluations
    global countFE;
    countFE = 0 ;

    %% Initialize iteration counter 
    N_iter = 0;

    %% Goal reached flag
    goalReached = false; 

    %% Initialize Global best
    GlobalBest.Position = [];
    GlobalBest.Cost = Inf ; 

    %% Set the seed for random number generator
    rng('default');  % Resets to the default settings
    rng('shuffle'); % set it to shuffle
    
    %% Initialize population and update the global best
    %% create empty moth
    moth.Position = [];
    moth.Velocity = [];
    moth.Cost = Inf;
    moth.Best.Position = [];
    moth.Best.Cost = Inf;

    % Initialize moths array
    population = repmat(moth, nPop, 1);

    for i = 1:nPop
        % Initialize Position
        population(i).Position = unifrnd(lb, ub, [ 1, nd] );  

        %initilaize velocities
        population(i).Velocity = zeros([1, nd]);

        % Evaluation of the cost
        population(i).Cost = costFunction(population(i).Position);
          
        % Update Personal Best
        population(i).Best.Position = population(i).Position;
        population(i).Best.Cost = population(i).Cost;

        % Update Global Best
        if population(i).Best.Cost < GlobalBest.Cost
            GlobalBest = population(i).Best;
        end
    end

    %% Velocity Limits
    VelMax = 0.1 * (ub - lb);
    VelMin= - VelMax;

   %% begin algorithm loop 
    while (GlobalBest.Cost > tol)  && (countFE <= maxfe)
        %% update the generation
        N_iter=N_iter+1; 

        for i = 1:nPop
             %% Update velocity
            population(i).Velocity = w*population(i).Velocity ...
                                    + c1*rand([1,nd]).*(population(i).Best.Position - population(i).Position) ...
                                    + c2*rand([1,nd]).*(GlobalBest.Position - population(i).Position);
            
            %% Apply Velocity Limits
            population(i).Velocity = max(population(i).Velocity,VelMin);
            population(i).Velocity = min(population(i).Velocity,VelMax);
    
            %% sort popuation for MFO 
            [~, SortOrder] = sort([population.Cost]);
            sorted_population = population(SortOrder);  
    
            %% Update Number of flames 
            Flame_no = round(nPop - countFE * ((nPop - 1) / maxfe));
            Flame_no = max(Flame_no, 1); % make sure it is always >= 1 becuase it will be used as an index for array
            
            %% Update Convergence Const a linearly dicreases from -1 to -2 
            %a= -1 + ACount*((-1)/ARef);
            a = aMax + (aMin - aMax) * (countFE/maxfe);
    
            %% Update position: divide it into two groups:
            %% group 1: update psotion using PSO
            if rand < 0.5  
               % size(population(i).Velocity)
                population(i).Position = population(i).Position + population(i).Velocity;
                % Velocity Mirror Effect
                IsOutside=(population(i).Position < lb | population(i).Position > ub);
                population(i).Velocity(IsOutside)= -population(i).Velocity(IsOutside);
            else
            %% group2: update position using MFO
                for d=1:nd
                    t=(a-1)*rand+1;
                    % Calculate the distance to flames (D) 
                    distance_to_flame=abs( sorted_population(i).Position(d) - population(i).Position(d));
    
                    if i<=Flame_no % Update the position of the moth with respect to its corresponsing flame                
                        F = sorted_population(i).Position(d); %  corresponsing flame to the moth
                    else % Upaate the position of the moth with respct to one flame 
                        F = sorted_population(Flame_no).Position(d); % the one flame
                    end
                     % update the moth position
                     population(i).Position(d) = distance_to_flame*exp(b.*t).*cos(t.*2*pi)+ F;
                end %dim  
    
            end

            %% Apply bounds/limits
            population(i).Position = max(population(i).Position, lb);
            population(i).Position = min(population(i).Position, ub);
            
            %% Evaluate
            population(i).Cost = costFunction(population(i).Position);   
            % Update Personal Best
            if population(i).Cost < population(i).Best.Cost
                population(i).Best.Position = population(i).Position;
                population(i).Best.Cost = population(i).Cost;

                % Update Global Best
                if population(i).Best.Cost < GlobalBest.Cost
                    GlobalBest = population(i).Best;
                end
            end
        end % end of population

        %% check if maxfes is exceeded 
        if countFE > maxfe 
            break;
        end

        %% print the iteration number
        if print_flag            
            fprintf('%s | CEC%d_F%d_D%d | Iteration %d |FEs %d | Error %d\n', AlgName, CECyear,  fNo , nd, N_iter, countFE,GlobalBest.Cost);
        end

         %check tolerance /error
        if (GlobalBest.Cost <= tol)
            GlobalBest.Cost  = 0 ;
            disp('tol reached');
            goalReached = true ; 
            break ;  % not needed, becuase it will exit in the next while loop check
        end
    end
end















