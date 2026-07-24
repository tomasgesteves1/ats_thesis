# Configuração do latexmk para compilar glossários (acrónimos) e nomenclaturas (símbolos)

# Adiciona dependências personalizadas para os glossários/acrónimos
add_cus_dep('acn', 'acr', 0, 'run_makeglossaries');
add_cus_dep('glo', 'gls', 0, 'run_makeglossaries');

sub run_makeglossaries {
    my ($base, $ext) = @_;
    if ( $silent ) {
        system "makeglossaries -q \"$base\"";
    } else {
        system "makeglossaries \"$base\"";
    }
}

# Adiciona dependência personalizada para a nomenclatura (símbolos)
add_cus_dep('nlo', 'nls', 0, 'run_makenlo2nls');

sub run_makenlo2nls {
    my ($base, $ext) = @_;
    system "makeindex \"$base.nlo\" -s nomencl.ist -o \"$base.nls\"";
}

# Adiciona extensões geradas à lista de limpeza do latexmk
push @generated_exts, 'acr', 'acn', 'alg', 'gls', 'glo', 'glg', 'nlo', 'nls', 'ilg';
