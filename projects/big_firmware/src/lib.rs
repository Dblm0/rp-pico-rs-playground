use proc_macro::{Literal, TokenStream, TokenTree};

#[proc_macro]
pub fn generate_blob(_item: TokenStream) -> TokenStream {
    let s = vec!["AAAAAAAAAA"; 173_500].join("\r\n");
    TokenTree::Literal(Literal::string(&s)).into()
}
