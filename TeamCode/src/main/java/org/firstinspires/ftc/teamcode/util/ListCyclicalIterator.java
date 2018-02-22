package org.firstinspires.ftc.teamcode.util;

import java.util.List;

public class ListCyclicalIterator<T> implements CyclicalIterator<T> {
    private final List<? extends T> items;
    private int index = 0;

    public ListCyclicalIterator(List<? extends T> items) {
        this.items = items;
    }

    @Override
    public T current() {
        index = index % items.size();
        return items.get(index);
    }

    @Override
    public T next() {
        index = (index + 1) % items.size();
        return current();
    }

    @Override
    public T prev() {
        if (--index < 0)
            index = items.size() + index;

        return current();
    }
}
