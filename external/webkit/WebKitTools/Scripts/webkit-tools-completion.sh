__webkit-patch_generate_reply()
{
    COMPREPLY=( $(compgen -W "$1" -- "${COMP_WORDS[COMP_CWORD]}") )
}

_webkit-patch_complete()
{
    local command current_command="${COMP_WORDS[1]}"
    case "$current_command" in
        -h|--help)
            command="help";
            ;;
        *)
            command="$current_command"
            ;;
    esac

    if [ $COMP_CWORD -eq 1 ]; then
        __webkit-patch_generate_reply "--help apply-from-bug bugs-to-commit commit-message land land-from-bug obsolete-attachments patches-to-commit post upload tree-status rollout reviewed-patches"
        return
    fi

    case "$command" in
        apply-from-bug)
            __webkit-patch_generate_reply "--force-clean --local-commit --no-clean --no-update"
            return
            ;;
        commit-message)
            return
            ;;
        land)
            __webkit-patch_generate_reply "--no-build --no-close --no-test --reviewer= -r"
            return
            ;;
        land-from-bug)
            __webkit-patch_generate_reply "--force-clean --no-build --no-clean --no-test"
            return
            ;;
        obsolete-attachments)
            return
            ;;
        post)
            __webkit-patch_generate_reply "--description --no-obsolete --no-review --request-commit -m --open-bug"
            return
            ;;
        upload)
            __webkit-patch_generate_reply "--description --no-obsolete --no-review --request-commit --cc -m --open-bug"
            return
            ;;
        post-commits)
            __webkit-patch_generate_reply "--bug-id= --no-comment --no-obsolete --no-review -b"
            return
            ;;
    esac
}

complete -F _webkit-patch_complete webkit-patch
complete -W "--continue --fix-merged --help --no-continue --no-warnings --warnings -c -f -h -w" resolve-ChangeLogs
complete -W "--bug --diff --git-commit --git-index --git-reviewer --help --no-update --no-write --open --update --write -d -h -o" prepare-ChangeLog
complete -W "--clean --debug --help -h" build-webkit
complete -o default -W "--add-platform-exceptions --complex-text --configuration --guard-malloc --help --http --ignore-tests --launch-safari --leaks --merge-leak-depth --new-test-results --no-http --no-launch-safari --no-new-test-results --no-sample-on-timeout --no-strip-editing-callbacks --pixel-tests --platform --port --quiet --random --reset-results --results-directory --reverse --root --sample-on-timeout --singly --skipped --slowest --strict --strip-editing-callbacks --threaded --timeout --tolerance --use-remote-links-to-tests --valgrind --verbose -1 -c -g -h -i -l -m -o -p -q -t -v" run-webkit-tests
