function stylePlot(ax, lineWidth, fontSize, fontName, fileName)
%STYLEPLOT Applies consistent styling to MATLAB plots and optionally saves to PNG.
%
%   stylePlot(ax, lineWidth, fontSize, fontName, fileName)
%
%   - ax:        Axes handle (e.g., gca). If empty, uses current axes.
%   - lineWidth: Desired line width for all lines in the axes (default: 1.5)
%   - fontSize:  Font size for axes labels, ticks, and legend (default: 14)
%   - fontName:  Font name (e.g., 'Times New Roman', 'Helvetica') (default)
%   - fileName:  Optional string to save the figure as a PNG file
%
% Example:
%   plot(x, y);
%   stylePlot(gca, 2, 14, 'Times New Roman', 'myFigure.png');

    % --- Handle default arguments ---
    if nargin < 1 || isempty(ax)
        ax = gca;
    end
    if nargin < 2 || isempty(lineWidth)
        lineWidth = 1.5;
    end
    if nargin < 3 || isempty(fontSize)
        fontSize = 14;
    end
    if nargin < 4 || isempty(fontName)
        fontName = 'Times New Roman';
    end

    % --- Apply style to lines ---
    lines = findall(ax, 'Type', 'Line');
    for i = 1:numel(lines)
        lines(i).LineWidth = lineWidth;
    end

    % --- Style axes ---
    set(ax, 'FontSize', fontSize, ...
            'FontName', fontName, ...
            'TickLabelInterpreter', 'latex');

    % --- Style labels and title ---
    set(get(ax, 'XLabel'), 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
    set(get(ax, 'YLabel'), 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
    set(get(ax, 'ZLabel'), 'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);
    set(get(ax, 'Title'),  'Interpreter', 'latex', 'FontSize', fontSize, 'FontName', fontName);

    % --- Style legend(s) ---
    lgd = findobj(gcf, 'Type', 'Legend');
    for i = 1:numel(lgd)
        set(lgd(i), 'Interpreter', 'latex', ...
                    'FontSize', fontSize, ...
                    'FontName', fontName);
    end

    % --- Save to PNG if filename is provided ---
    if nargin >= 5 && ~isempty(fileName)
        % Ensure it ends with .png
        [~, ~, ext] = fileparts(fileName);
        if isempty(ext)
            fileName = [fileName '.png'];
        end
        % Set figure background to white for clean export
        set(gcf, 'Color', 'w');
        try
            exportgraphics(gcf, fileName, 'Resolution', 300);
        catch
            print(gcf, fileName, '-dpng', '-r300');
        end
        fprintf('✅ Figure saved to "%s"\n', fileName);
    end
end