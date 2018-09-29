package org.firstinspires.ftc.teamcode.util;

/**
 * Cycle through different items
 */
public interface CyclicalIterator<T> {
    T current();
    T next();
    T prev();
}
