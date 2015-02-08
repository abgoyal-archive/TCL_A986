
/*      $NetBSD: queue.h,v 1.52 2009/04/20 09:56:08 mschuett Exp $ */



#ifndef QEMU_SYS_QUEUE_H_
#define QEMU_SYS_QUEUE_H_


#define QLIST_HEAD(name, type)                                          \
struct name {                                                           \
        struct type *lh_first;  /* first element */                     \
}

#define QLIST_HEAD_INITIALIZER(head)                                    \
        { NULL }

#define QLIST_ENTRY(type)                                               \
struct {                                                                \
        struct type *le_next;   /* next element */                      \
        struct type **le_prev;  /* address of previous next element */  \
}

#define QLIST_INIT(head) do {                                           \
        (head)->lh_first = NULL;                                        \
} while (/*CONSTCOND*/0)

#define QLIST_INSERT_AFTER(listelm, elm, field) do {                    \
        if (((elm)->field.le_next = (listelm)->field.le_next) != NULL)  \
                (listelm)->field.le_next->field.le_prev =               \
                    &(elm)->field.le_next;                              \
        (listelm)->field.le_next = (elm);                               \
        (elm)->field.le_prev = &(listelm)->field.le_next;               \
} while (/*CONSTCOND*/0)

#define QLIST_INSERT_BEFORE(listelm, elm, field) do {                   \
        (elm)->field.le_prev = (listelm)->field.le_prev;                \
        (elm)->field.le_next = (listelm);                               \
        *(listelm)->field.le_prev = (elm);                              \
        (listelm)->field.le_prev = &(elm)->field.le_next;               \
} while (/*CONSTCOND*/0)

#define QLIST_INSERT_HEAD(head, elm, field) do {                        \
        if (((elm)->field.le_next = (head)->lh_first) != NULL)          \
                (head)->lh_first->field.le_prev = &(elm)->field.le_next;\
        (head)->lh_first = (elm);                                       \
        (elm)->field.le_prev = &(head)->lh_first;                       \
} while (/*CONSTCOND*/0)

#define QLIST_REMOVE(elm, field) do {                                   \
        if ((elm)->field.le_next != NULL)                               \
                (elm)->field.le_next->field.le_prev =                   \
                    (elm)->field.le_prev;                               \
        *(elm)->field.le_prev = (elm)->field.le_next;                   \
} while (/*CONSTCOND*/0)

#define QLIST_FOREACH(var, head, field)                                 \
        for ((var) = ((head)->lh_first);                                \
                (var);                                                  \
                (var) = ((var)->field.le_next))

#define QLIST_FOREACH_SAFE(var, head, field, next_var)                  \
        for ((var) = ((head)->lh_first);                                \
                (var) && ((next_var) = ((var)->field.le_next), 1);      \
                (var) = (next_var))

#define QLIST_EMPTY(head)                ((head)->lh_first == NULL)
#define QLIST_FIRST(head)                ((head)->lh_first)
#define QLIST_NEXT(elm, field)           ((elm)->field.le_next)


#define QSIMPLEQ_HEAD(name, type)                                       \
struct name {                                                           \
    struct type *sqh_first;    /* first element */                      \
    struct type **sqh_last;    /* addr of last next element */          \
}

#define QSIMPLEQ_HEAD_INITIALIZER(head)                                 \
    { NULL, &(head).sqh_first }

#define QSIMPLEQ_ENTRY(type)                                            \
struct {                                                                \
    struct type *sqe_next;    /* next element */                        \
}

#define QSIMPLEQ_INIT(head) do {                                        \
    (head)->sqh_first = NULL;                                           \
    (head)->sqh_last = &(head)->sqh_first;                              \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_INSERT_HEAD(head, elm, field) do {                     \
    if (((elm)->field.sqe_next = (head)->sqh_first) == NULL)            \
        (head)->sqh_last = &(elm)->field.sqe_next;                      \
    (head)->sqh_first = (elm);                                          \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_INSERT_TAIL(head, elm, field) do {                     \
    (elm)->field.sqe_next = NULL;                                       \
    *(head)->sqh_last = (elm);                                          \
    (head)->sqh_last = &(elm)->field.sqe_next;                          \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_INSERT_AFTER(head, listelm, elm, field) do {           \
    if (((elm)->field.sqe_next = (listelm)->field.sqe_next) == NULL)    \
        (head)->sqh_last = &(elm)->field.sqe_next;                      \
    (listelm)->field.sqe_next = (elm);                                  \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_REMOVE_HEAD(head, field) do {                          \
    if (((head)->sqh_first = (head)->sqh_first->field.sqe_next) == NULL)\
        (head)->sqh_last = &(head)->sqh_first;                          \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_REMOVE(head, elm, type, field) do {                    \
    if ((head)->sqh_first == (elm)) {                                   \
        QSIMPLEQ_REMOVE_HEAD((head), field);                            \
    } else {                                                            \
        struct type *curelm = (head)->sqh_first;                        \
        while (curelm->field.sqe_next != (elm))                         \
            curelm = curelm->field.sqe_next;                            \
        if ((curelm->field.sqe_next =                                   \
            curelm->field.sqe_next->field.sqe_next) == NULL)            \
                (head)->sqh_last = &(curelm)->field.sqe_next;           \
    }                                                                   \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_FOREACH(var, head, field)                              \
    for ((var) = ((head)->sqh_first);                                   \
        (var);                                                          \
        (var) = ((var)->field.sqe_next))

#define QSIMPLEQ_FOREACH_SAFE(var, head, field, next)                   \
    for ((var) = ((head)->sqh_first);                                   \
        (var) && ((next = ((var)->field.sqe_next)), 1);                 \
        (var) = (next))

#define QSIMPLEQ_CONCAT(head1, head2) do {                              \
    if (!QSIMPLEQ_EMPTY((head2))) {                                     \
        *(head1)->sqh_last = (head2)->sqh_first;                        \
        (head1)->sqh_last = (head2)->sqh_last;                          \
        QSIMPLEQ_INIT((head2));                                         \
    }                                                                   \
} while (/*CONSTCOND*/0)

#define QSIMPLEQ_LAST(head, type, field)                                \
    (QSIMPLEQ_EMPTY((head)) ?                                           \
        NULL :                                                          \
            ((struct type *)(void *)                                    \
        ((char *)((head)->sqh_last) - offsetof(struct type, field))))

#define QSIMPLEQ_EMPTY(head)        ((head)->sqh_first == NULL)
#define QSIMPLEQ_FIRST(head)        ((head)->sqh_first)
#define QSIMPLEQ_NEXT(elm, field)   ((elm)->field.sqe_next)


#define Q_TAILQ_HEAD(name, type, qual)                                  \
struct name {                                                           \
        qual type *tqh_first;           /* first element */             \
        qual type *qual *tqh_last;      /* addr of last next element */ \
}
#define QTAILQ_HEAD(name, type)  Q_TAILQ_HEAD(name, struct type,)

#define QTAILQ_HEAD_INITIALIZER(head)                                   \
        { NULL, &(head).tqh_first }

#define Q_TAILQ_ENTRY(type, qual)                                       \
struct {                                                                \
        qual type *tqe_next;            /* next element */              \
        qual type *qual *tqe_prev;      /* address of previous next element */\
}
#define QTAILQ_ENTRY(type)       Q_TAILQ_ENTRY(struct type,)

#define QTAILQ_INIT(head) do {                                          \
        (head)->tqh_first = NULL;                                       \
        (head)->tqh_last = &(head)->tqh_first;                          \
} while (/*CONSTCOND*/0)

#define QTAILQ_INSERT_HEAD(head, elm, field) do {                       \
        if (((elm)->field.tqe_next = (head)->tqh_first) != NULL)        \
                (head)->tqh_first->field.tqe_prev =                     \
                    &(elm)->field.tqe_next;                             \
        else                                                            \
                (head)->tqh_last = &(elm)->field.tqe_next;              \
        (head)->tqh_first = (elm);                                      \
        (elm)->field.tqe_prev = &(head)->tqh_first;                     \
} while (/*CONSTCOND*/0)

#define QTAILQ_INSERT_TAIL(head, elm, field) do {                       \
        (elm)->field.tqe_next = NULL;                                   \
        (elm)->field.tqe_prev = (head)->tqh_last;                       \
        *(head)->tqh_last = (elm);                                      \
        (head)->tqh_last = &(elm)->field.tqe_next;                      \
} while (/*CONSTCOND*/0)

#define QTAILQ_INSERT_AFTER(head, listelm, elm, field) do {             \
        if (((elm)->field.tqe_next = (listelm)->field.tqe_next) != NULL)\
                (elm)->field.tqe_next->field.tqe_prev =                 \
                    &(elm)->field.tqe_next;                             \
        else                                                            \
                (head)->tqh_last = &(elm)->field.tqe_next;              \
        (listelm)->field.tqe_next = (elm);                              \
        (elm)->field.tqe_prev = &(listelm)->field.tqe_next;             \
} while (/*CONSTCOND*/0)

#define QTAILQ_INSERT_BEFORE(listelm, elm, field) do {                  \
        (elm)->field.tqe_prev = (listelm)->field.tqe_prev;              \
        (elm)->field.tqe_next = (listelm);                              \
        *(listelm)->field.tqe_prev = (elm);                             \
        (listelm)->field.tqe_prev = &(elm)->field.tqe_next;             \
} while (/*CONSTCOND*/0)

#define QTAILQ_REMOVE(head, elm, field) do {                            \
        if (((elm)->field.tqe_next) != NULL)                            \
                (elm)->field.tqe_next->field.tqe_prev =                 \
                    (elm)->field.tqe_prev;                              \
        else                                                            \
                (head)->tqh_last = (elm)->field.tqe_prev;               \
        *(elm)->field.tqe_prev = (elm)->field.tqe_next;                 \
} while (/*CONSTCOND*/0)

#define QTAILQ_FOREACH(var, head, field)                                \
        for ((var) = ((head)->tqh_first);                               \
                (var);                                                  \
                (var) = ((var)->field.tqe_next))

#define QTAILQ_FOREACH_SAFE(var, head, field, next_var)                 \
        for ((var) = ((head)->tqh_first);                               \
                (var) && ((next_var) = ((var)->field.tqe_next), 1);     \
                (var) = (next_var))

#define QTAILQ_FOREACH_REVERSE(var, head, headname, field)              \
        for ((var) = (*(((struct headname *)((head)->tqh_last))->tqh_last));    \
                (var);                                                  \
                (var) = (*(((struct headname *)((var)->field.tqe_prev))->tqh_last)))

#define QTAILQ_EMPTY(head)               ((head)->tqh_first == NULL)
#define QTAILQ_FIRST(head)               ((head)->tqh_first)
#define QTAILQ_NEXT(elm, field)          ((elm)->field.tqe_next)

#define QTAILQ_LAST(head, headname) \
        (*(((struct headname *)((head)->tqh_last))->tqh_last))
#define QTAILQ_PREV(elm, headname, field) \
        (*(((struct headname *)((elm)->field.tqe_prev))->tqh_last))


#define QCIRCLEQ_HEAD(name, type)                                       \
struct name {                                                           \
        struct type *cqh_first;         /* first element */             \
        struct type *cqh_last;          /* last element */              \
}

#define QCIRCLEQ_HEAD_INITIALIZER(head)                                 \
        { (void *)&head, (void *)&head }

#define QCIRCLEQ_ENTRY(type)                                            \
struct {                                                                \
        struct type *cqe_next;          /* next element */              \
        struct type *cqe_prev;          /* previous element */          \
}

#define QCIRCLEQ_INIT(head) do {                                        \
        (head)->cqh_first = (void *)(head);                             \
        (head)->cqh_last = (void *)(head);                              \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_INSERT_AFTER(head, listelm, elm, field) do {           \
        (elm)->field.cqe_next = (listelm)->field.cqe_next;              \
        (elm)->field.cqe_prev = (listelm);                              \
        if ((listelm)->field.cqe_next == (void *)(head))                \
                (head)->cqh_last = (elm);                               \
        else                                                            \
                (listelm)->field.cqe_next->field.cqe_prev = (elm);      \
        (listelm)->field.cqe_next = (elm);                              \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_INSERT_BEFORE(head, listelm, elm, field) do {          \
        (elm)->field.cqe_next = (listelm);                              \
        (elm)->field.cqe_prev = (listelm)->field.cqe_prev;              \
        if ((listelm)->field.cqe_prev == (void *)(head))                \
                (head)->cqh_first = (elm);                              \
        else                                                            \
                (listelm)->field.cqe_prev->field.cqe_next = (elm);      \
        (listelm)->field.cqe_prev = (elm);                              \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_INSERT_HEAD(head, elm, field) do {                     \
        (elm)->field.cqe_next = (head)->cqh_first;                      \
        (elm)->field.cqe_prev = (void *)(head);                         \
        if ((head)->cqh_last == (void *)(head))                         \
                (head)->cqh_last = (elm);                               \
        else                                                            \
                (head)->cqh_first->field.cqe_prev = (elm);              \
        (head)->cqh_first = (elm);                                      \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_INSERT_TAIL(head, elm, field) do {                     \
        (elm)->field.cqe_next = (void *)(head);                         \
        (elm)->field.cqe_prev = (head)->cqh_last;                       \
        if ((head)->cqh_first == (void *)(head))                        \
                (head)->cqh_first = (elm);                              \
        else                                                            \
                (head)->cqh_last->field.cqe_next = (elm);               \
        (head)->cqh_last = (elm);                                       \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_REMOVE(head, elm, field) do {                          \
        if ((elm)->field.cqe_next == (void *)(head))                    \
                (head)->cqh_last = (elm)->field.cqe_prev;               \
        else                                                            \
                (elm)->field.cqe_next->field.cqe_prev =                 \
                    (elm)->field.cqe_prev;                              \
        if ((elm)->field.cqe_prev == (void *)(head))                    \
                (head)->cqh_first = (elm)->field.cqe_next;              \
        else                                                            \
                (elm)->field.cqe_prev->field.cqe_next =                 \
                    (elm)->field.cqe_next;                              \
} while (/*CONSTCOND*/0)

#define QCIRCLEQ_FOREACH(var, head, field)                              \
        for ((var) = ((head)->cqh_first);                               \
                (var) != (const void *)(head);                          \
                (var) = ((var)->field.cqe_next))

#define QCIRCLEQ_FOREACH_REVERSE(var, head, field)                      \
        for ((var) = ((head)->cqh_last);                                \
                (var) != (const void *)(head);                          \
                (var) = ((var)->field.cqe_prev))

#define QCIRCLEQ_EMPTY(head)             ((head)->cqh_first == (void *)(head))
#define QCIRCLEQ_FIRST(head)             ((head)->cqh_first)
#define QCIRCLEQ_LAST(head)              ((head)->cqh_last)
#define QCIRCLEQ_NEXT(elm, field)        ((elm)->field.cqe_next)
#define QCIRCLEQ_PREV(elm, field)        ((elm)->field.cqe_prev)

#define QCIRCLEQ_LOOP_NEXT(head, elm, field)                            \
        (((elm)->field.cqe_next == (void *)(head))                      \
            ? ((head)->cqh_first)                                       \
            : (elm->field.cqe_next))
#define QCIRCLEQ_LOOP_PREV(head, elm, field)                            \
        (((elm)->field.cqe_prev == (void *)(head))                      \
            ? ((head)->cqh_last)                                        \
            : (elm->field.cqe_prev))

#endif  /* !QEMU_SYS_QUEUE_H_ */
