% Example usage:
data = results;

generateLatexTable(data, 3, 'latexTable.tex');

function generateLatexTable(data, sigDigits, outputFile)
    % Rounds the data to the specified number of significant digits
    roundedData = round(data, sigDigits, 'significant');

    % Open file to write
    fileID = fopen(outputFile, 'w');
    
    fprintf(fileID, '\\begin{table}[h!]\n');
    fprintf(fileID, '    \\centering\n');
    fprintf(fileID, '    \\resizebox{\\linewidth}{!}{\n');
    fprintf(fileID, '    \\begin{tabular}{ccccccccccc}\n');
    fprintf(fileID, '        \\toprule\n');
    fprintf(fileID, '        \\text{Weights} & \\multicolumn{6}{c}{} & \\text{Response} & \\multicolumn{5}{c}{} \\\\\n');
    fprintf(fileID, '        \\cmidrule(lr){1-6} \\cmidrule(lr){7-11}\n');
    fprintf(fileID, '        & \\(\\alpha_{h1}\\) & \\(\\zeta^p_{h1}\\) & \\(\\alpha_{h2}\\) & \\(\\zeta^p_{h2}\\) & \\(\\kappa\\) &  \\(w_{a,1}^o\\) & \\(v^p_{a,1}\\) & \\(w_{a,2}^o\\) & \\(v^p_{a,2}\\) & \\(\\Delta\\) \\\\\n');
    fprintf(fileID, '        \\midrule\n');
    
    % Iterate through each row of the data
    [rows, cols] = size(roundedData);
    for i = 1:rows
        fprintf(fileID, '        %d & ', i);
        for j = 1:cols
            if j == cols
                fprintf(fileID, '%.4g', roundedData(i, j));
            else
                fprintf(fileID, '%.4g & ', roundedData(i, j));
            end
        end
        fprintf(fileID, ' \\\\\n');
    end

    fprintf(fileID, '        \\bottomrule\n');
    fprintf(fileID, '    \\end{tabular}\n');
    fprintf(fileID, '    }\n');
    fprintf(fileID, '    \\caption{Full Factorial Design for DOE (32 Experiments)}\n');
    fprintf(fileID, '    \\label{tab:experiments}\n');
    fprintf(fileID, '\\end{table}\n');
    
    % Close the file
    fclose(fileID);

    disp(['LaTeX table written to ', outputFile]);
end


