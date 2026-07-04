% SPDX-License-Identifier: AGPL-3.0-only
%
% Generate MF6.1 golden CSVs for the Pacejka book reference tyre using an external Magic-Formula
% implementation as a NUMERICAL ORACLE (outputs used as data only — the oracle's source is never
% read, ported, or vendored; outlap's MF6.1 is derived from the Pacejka book alone). This script
% is our own authorship.
%
% Oracle: teasit/magic-formula-tyre-library (GPL-3.0), package `magicformula.v61`. We call the
% package function `magicformula.v61.eval` directly with a parameter struct, bypassing the
% object/.tir reader. GPL is fine here: the library is executed as a tool and only its numeric
% outputs are captured.
%
% Environment:
%   MF_ORACLE_PKG : directory placed on the Octave path that CONTAINS the `+magicformula` package
%                   (i.e. a copy of the library's `src/` with the top-level `magicformula.m`
%                   function omitted, so it does not shadow the `+magicformula` package).
%   MF_GOLDEN_OUT : output directory for the CSV files.
%   MF_ORACLE_TAG : provenance string for the oracle (name + version/commit).
%
% Run via tools/goldens/run.sh. Never runs in CI — the committed CSVs are compared there.

function gen_mf61_goldens()
  pkg_dir = getenv('MF_ORACLE_PKG');
  out_dir = getenv('MF_GOLDEN_OUT');
  oracle_tag = getenv('MF_ORACLE_TAG');
  if isempty(pkg_dir) || isempty(out_dir)
    error('set MF_ORACLE_PKG and MF_GOLDEN_OUT');
  end
  % Require a commit-pinned oracle tag so committed goldens are never anonymously regenerated
  % (a test rejects headers without `oracle: ... @ <hash>`). run.sh fills this in.
  if isempty(oracle_tag) || isempty(strfind(oracle_tag, '@'))
    error('set MF_ORACLE_TAG to the oracle name + commit, e.g. "teasit ... @ <sha> (GPL-3.0)"');
  end
  addpath(pkg_dir);
  if exist(fullfile(pkg_dir, 'enum'), 'dir'); addpath(fullfile(pkg_dir, 'enum')); end

  p = book_params();
  octver = OCTAVE_VERSION;
  header = sprintf(['# generator: tools/goldens/gen_mf61_goldens.m, oracle: %s, GNU Octave %s\n' ...
                    '# tyre: pacejka_2006_205_60r15 (Pacejka 2006 Table A3.1), ISO 8855 sign\n'], ...
                   oracle_tag, octver);

  FZ0 = p.FNOMIN; IP = p.NOMPRES; VX = p.LONGVL;
  cols = 'kappa,alpha_rad,gamma_rad,fz_n,p_pa,vx_mps,fx_n,fy_n,mz_nm,mx_nm,my_nm\n';

  % --- fx0.csv: pure longitudinal, kappa x Fz (alpha = 0, gamma = 0) ---
  kap = linspace(-0.30, 0.30, 41);
  fzs = [0.5 1.0 1.5 2.0] * FZ0;
  rows = [];
  for fz = fzs
    for k = kap
      rows = [rows; eval_row(p, k, 0.0, 0.0, fz, IP, VX)];
    end
  end
  write_csv(fullfile(out_dir, 'fx0.csv'), header, cols, rows);

  % --- fy0_mz.csv: pure lateral, alpha x Fz x gamma (kappa = 0) ---
  alp = linspace(-0.21, 0.21, 41);
  gam = [-4 0 4] * pi/180;
  rows = [];
  for fz = fzs
    for g = gam
      for a = alp
        rows = [rows; eval_row(p, 0.0, a, g, fz, IP, VX)];
      end
    end
  end
  write_csv(fullfile(out_dir, 'fy0_mz.csv'), header, cols, rows);

  % --- combined.csv: kappa x alpha at nominal load, gamma = 0 ---
  kc = linspace(-0.25, 0.25, 11);
  ac = linspace(-0.17, 0.17, 11);
  rows = [];
  for k = kc
    for a = ac
      rows = [rows; eval_row(p, k, a, 0.0, FZ0, IP, VX)];
    end
  end
  write_csv(fullfile(out_dir, 'combined.csv'), header, cols, rows);

  % --- combined_camber.csv: kappa x alpha at gamma = ±4° — couples combined slip AND camber,
  %     the exact regime the Mz zero-camber-trail + s·Fx fixes govern (κ≠0 ∧ γ≠0). ---
  gam2 = [-4, 4] * pi/180;
  rows = [];
  for g = gam2
    for k = kc
      for a = ac
        rows = [rows; eval_row(p, k, a, g, FZ0, IP, VX)];
      end
    end
  end
  write_csv(fullfile(out_dir, 'combined_camber.csv'), header, cols, rows);

  % Note: pressure, forward speed, and running direction are held nominal (p = NOMPRES, V = +LONGVL).
  % The 2nd-edition reference set has no PP* pressure terms and no QSY3/4 speed terms, so those
  % sweeps would exercise nothing; add them when a tyre with those sensitivities ships.

  printf('wrote goldens to %s\n', out_dir);
end

% Evaluate the oracle at one operating point and return a contract row.
function row = eval_row(p, kappa, alpha, gamma, fz, ip, vx)
  [FX, FY, MZ, MY, MX] = magicformula.v61.eval(p, kappa, alpha, fz, ip, gamma, vx, 0);
  row = [kappa, alpha, gamma, fz, ip, vx, FX, FY, MZ, MX, MY];
end

function write_csv(path, header, cols, rows)
  fid = fopen(path, 'w');
  fprintf(fid, '%s', header);
  fprintf(fid, cols);
  for i = 1:size(rows, 1)
    fprintf(fid, '%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g\n', rows(i, :));
  end
  fclose(fid);
end

% Full MF6.1 parameter struct: every field the oracle references, defaulted (0, L*=1, PKY4=2,
% PKY2=1, TYRESIDE=0), then overlaid with the Pacejka 2006 Table A3.1 values (205/60R15). Any
% transcription drift versus data/tires/pacejka_2006_205_60r15/car.tyr.yaml is caught by the Rust
% golden test (our model reads the .tyr; this oracle reads the struct — they must agree).
function p = book_params()
  fields = { ...
    'FNOMIN','LCX','LCY','LEX','LEY','LFZO','LHX','LHY','LKX','LKY','LKYC','LKZC','LMUX','LMUY', ...
    'LMX','LMY','LONGVL','LRES','LS','LTR','LVMX','LVX','LVY','LVYKA','LXAL','LYKA','NOMPRES', ...
    'PCX1','PCY1','PDX1','PDX2','PDX3','PDY1','PDY2','PDY3','PEX1','PEX2','PEX3','PEX4','PEY1', ...
    'PEY2','PEY3','PEY4','PEY5','PHX1','PHX2','PHY1','PHY2','PKX1','PKX2','PKX3','PKY1','PKY2', ...
    'PKY3','PKY4','PKY5','PKY6','PKY7','PPMX1','PPX1','PPX2','PPX3','PPX4','PPY1','PPY2','PPY3', ...
    'PPY4','PPY5','PPZ1','PPZ2','PVX1','PVX2','PVY1','PVY2','PVY3','PVY4','QBZ1','QBZ10','QBZ2', ...
    'QBZ3','QBZ4','QBZ5','QBZ9','QCZ1','QDZ1','QDZ10','QDZ11','QDZ2','QDZ3','QDZ4','QDZ6','QDZ7', ...
    'QDZ8','QDZ9','QEZ1','QEZ2','QEZ3','QEZ4','QEZ5','QHZ1','QHZ2','QHZ3','QHZ4','QSX1','QSX10', ...
    'QSX11','QSX2','QSX3','QSX4','QSX5','QSX6','QSX7','QSX8','QSX9','QSY1','QSY2','QSY3','QSY4', ...
    'QSY5','QSY6','QSY7','QSY8','RBX1','RBX2','RBX3','RBY1','RBY2','RBY3','RBY4','RCX1','RCY1', ...
    'REX1','REX2','REY1','REY2','RHX1','RHY1','RHY2','RVY1','RVY2','RVY3','RVY4','RVY5','RVY6', ...
    'SSZ1','SSZ2','SSZ3','SSZ4','TYRESIDE','UNLOADED_RADIUS'};
  p = struct();
  for i = 1:numel(fields); p.(fields{i}) = 0; end
  % Scaling factors default to unity.
  Ls = {'LCX','LCY','LEX','LEY','LFZO','LHX','LHY','LKX','LKY','LKYC','LKZC','LMUX','LMUY','LMX', ...
        'LMY','LRES','LS','LTR','LVMX','LVX','LVY','LVYKA','LXAL','LYKA'};
  for i = 1:numel(Ls); p.(Ls{i}) = 1; end
  p.PKY4 = 2; p.PKY2 = 1; p.TYRESIDE = 0;

  % --- Pacejka 2006 Table A3.1 (205/60R15 91V, 2.2 bar, ISO) ---
  p.FNOMIN = 4000; p.UNLOADED_RADIUS = 0.313; p.LONGVL = 16.67; p.NOMPRES = 220000;
  p.PCX1 = 1.685; p.PDX1 = 1.210; p.PDX2 = -0.037; p.PEX1 = 0.344; p.PEX2 = 0.095;
  p.PEX3 = -0.020; p.PEX4 = 0; p.PKX1 = 21.51; p.PKX2 = -0.163; p.PKX3 = 0.245;
  p.PHX1 = -0.002; p.PHX2 = 0.002; p.PVX1 = 0; p.PVX2 = 0;
  p.RBX1 = 12.35; p.RBX2 = -10.77; p.RBX3 = 0; p.RCX1 = 1.092; p.RHX1 = 0.007;
  p.PCY1 = 1.193; p.PDY1 = -0.990; p.PDY2 = 0.145; p.PDY3 = -11.23; p.PEY1 = -1.003;
  p.PEY2 = -0.537; p.PEY3 = -0.083; p.PEY4 = -4.787; p.PKY1 = -14.95; p.PKY2 = 2.130;
  p.PKY3 = -0.028; p.PKY4 = 2; p.PKY5 = 0; p.PKY6 = -0.92; p.PKY7 = -0.24;
  p.PHY1 = 0.003; p.PHY2 = -0.001; p.PVY1 = 0.045; p.PVY2 = -0.024; p.PVY3 = -0.532; p.PVY4 = 0.039;
  p.RBY1 = 6.461; p.RBY2 = 4.196; p.RBY3 = -0.015; p.RBY4 = 0; p.RCY1 = 1.081; p.RHY1 = 0.009;
  p.RVY1 = 0.053; p.RVY2 = -0.073; p.RVY3 = 0.517; p.RVY4 = 35.44; p.RVY5 = 1.9; p.RVY6 = -10.71;
  p.QBZ1 = 8.964; p.QBZ2 = -1.106; p.QBZ3 = -0.842; p.QBZ5 = -0.227; p.QBZ9 = 18.47; p.QBZ10 = 0;
  p.QCZ1 = 1.180; p.QDZ1 = 0.100; p.QDZ2 = -0.001; p.QDZ3 = 0.007; p.QDZ4 = 13.05;
  p.QDZ6 = -0.008; p.QDZ7 = 0.000; p.QDZ8 = -0.296; p.QDZ9 = -0.009;
  p.QEZ1 = -1.609; p.QEZ2 = -0.359; p.QEZ4 = 0.174; p.QEZ5 = -0.896;
  p.QHZ1 = 0.007; p.QHZ2 = -0.002; p.QHZ3 = 0.147; p.QHZ4 = 0.004;
  p.SSZ1 = 0.043; p.SSZ2 = 0.001; p.SSZ3 = 0.731; p.SSZ4 = -0.238;
  p.QSY1 = 0.01;
end
