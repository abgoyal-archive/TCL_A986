#!/bin/bash
self=$0

die_usage() {
    echo "Usage: $self <example.txt>"
    exit 1
}

die() {
    echo "$@"
    exit 1
}

include_block() {
    local on_block
    while IFS=$'\n' read -r t_line; do
        case "$t_line" in
            \~*\ ${block_name})
                if [ "x$on_block" == "xyes" ]; then
                    return 0;
                else
                    on_block=yes
                fi
                ;;
            *)
                if [ "x$on_block" == "xyes" ]; then
                    echo "$t_line"
                fi
                ;;
        esac
    done
    echo "WARNING: failed to find text for block $block_name" >&2
    return 1
}

txt=$1
[ -f "$txt" ] || die_usage
read -r template < "$txt"
case "$template" in
    @TEMPLATE*) template=${txt%/*}/${template##@TEMPLATE } ;;
    *) die "Failed to parse template name from '$template'" ;;
esac

fence="~~~~~~~~~"
fence="${fence}${fence}"
fence="${fence}${fence}"
fence="${fence}${fence}"
while IFS=$'\n' read -r line; do
    case "$line" in
        @TEMPLATE*)
            template=${template##@TEMPLATE }
            template=${template%.c}.txt
            ;;
        @DEFAULT)
            include_block < "$template"
            ;;
        ~~~*)
            block_name=${line##~* }
            [ "$block_name" == "INTRODUCTION" ] || echo "$fence"
            ;;
        *)  echo "$line"
            ;;
    esac
done < "$txt"

echo
echo "Putting It All Together"
echo "======================="
echo "${fence}"
${self%/*}/gen_example_code.sh "${txt}"
echo "${fence}"
