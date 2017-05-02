/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef __OpenKarto_Event_h__
#define __OpenKarto_Event_h__

#include <OpenKarto/List.h>
#include <OpenKarto/Mutex.h>

// Forward declaration for tbb::mutex
namespace tbb
{
  class mutex;
}

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // The following code is from the Poco library with the following license 
  // and modified to fit into the KartoSDK

  // Copyright (c) 2006, Applied Informatics Software Engineering GmbH.
  // and Contributors.
  //
  // Permission is hereby granted, free of charge, to any person or organization
  // obtaining a copy of the software and accompanying documentation covered by
  // this license (the "Software") to use, reproduce, display, distribute,
  // execute, and transmit the Software, and to prepare derivative works of the
  // Software, and to permit third-parties to whom the Software is furnished to
  // do so, all subject to the following:
  // 
  // The copyright notices in the Software and this entire statement, including
  // the above license grant, this restriction and the following disclaimer,
  // must be included in all copies of the Software, in whole or in part, and
  // all derivative works of the Software, unless such copies or derivative
  // works are solely in the form of machine-executable object code generated by
  // a source language processor.
  // 
  // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  // FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
  // SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
  // FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
  // ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  // DEALINGS IN THE SOFTWARE.

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  /**
   * Interface for Delegate and Expire
   * Very similar to AbstractPriorityDelegate but having two separate files (no inheritance)
   * allows one to have compile-time checks when registering an observer
   * instead of run-time checks.
   */
  template <class TArgs> 
  class AbstractDelegate
  {
  public:
    /** 
     * Construct an AbstractDelegate with a target
     * @param pTarget
     */
    AbstractDelegate(void* pTarget)
      : m_pTarget(pTarget)
    {
      assert(m_pTarget != 0);
    }

    /** 
     * Copy constructor. Creates the AbstractDelegate from another one.
     * @param rOther
     */
    AbstractDelegate(const AbstractDelegate& rOther)
      : m_pTarget(rOther.m_pTarget)
    {
      assert(m_pTarget != 0);
    }

    /** 
     * Destructor
     */
    virtual ~AbstractDelegate() 
    {
    }

  public:
    /**
     * Returns false, if the Delegate is no longer valid, thus indicating an expire.
     * @param pSender
     * @param rArguments
     */
    virtual void Notify(const void* pSender, TArgs& rArguments) = 0;

    /**
     * Returns a deep-copy of the AbstractDelegate
     * @return AbstractDelegate a clone of this 
     */
    virtual AbstractDelegate* Clone() const = 0;

    /**
     * Gets the target
     */
    void* GetTarget() const
    {
      return m_pTarget;
    }

  public:
    /**
     * Comparing AbstractDelegates in a collection.
     * @param rOther
     * @return true if equal 
     */
    kt_bool operator==(const AbstractDelegate<TArgs>& rOther) const
    {
      return m_pTarget == rOther.m_pTarget;
    }

    /**
     * Comparing AbstractDelegates in a collection.
     * @param rOther
     * @return true if less then 
     */
    kt_bool operator<(const AbstractDelegate<TArgs>& rOther) const
    {
      return m_pTarget < rOther.m_pTarget;
    }

  protected:
    /**
     * Target
     */
    void* m_pTarget;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template <class TObj, class TArgs, bool withSender=true> 
  class Delegate: public AbstractDelegate<TArgs>
  {
  public:
    typedef void (TObj::*NotifyMethod)(const void*, TArgs&);

    Delegate(TObj* pObject, NotifyMethod method)
      : AbstractDelegate<TArgs>(pObject)
      , m_ReceiverObject(pObject)
      , m_ReceiverMethod(method)
    {
    }

    /**
     * Copy constructor
     */
    Delegate(const Delegate& rDelegate)
      : AbstractDelegate<TArgs>(rDelegate)
      , m_ReceiverObject(rDelegate.m_ReceiverObject)
      , m_ReceiverMethod(rDelegate.m_ReceiverMethod)
    {
    }

    /**
     * Destructor
     */
    virtual ~Delegate()
    {
    }

  public:
    void Notify(const void* pSender, TArgs& rArguments)
    {
      (m_ReceiverObject->*m_ReceiverMethod)(pSender, rArguments);
    }

    AbstractDelegate<TArgs>* Clone() const
    {
      return new Delegate(*this);
    }

  public:
    /** 
     * Assignment operator. Assign another Delegate
     * @param rOther
     * @return reference to this Delegate
     */
    Delegate& operator=(const Delegate& rOther)
    {
      if (&rOther != this)
      {
        this->m_pTarget        = rOther.m_pTarget;
        this->m_ReceiverObject = rOther.m_ReceiverObject;
        this->m_ReceiverMethod = rOther.m_ReceiverMethod;
      }
      return *this;
    }

  protected:
    TObj*        m_ReceiverObject;
    NotifyMethod m_ReceiverMethod;

  private:
    Delegate();
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template <class TObj, class TArgs> 
  class Delegate<TObj, TArgs, false>: public AbstractDelegate<TArgs>
  {
  public:
    typedef void (TObj::*NotifyMethod)(TArgs&);

    Delegate(TObj* pObject, NotifyMethod method)
      : AbstractDelegate<TArgs>(pObject)
      , m_ReceiverObject(pObject)
      , m_ReceiverMethod(method)
    {
    }

    /**
     * Copy constructor
     */
    Delegate(const Delegate& rDelegate)
      : AbstractDelegate<TArgs>(rDelegate)
      , m_ReceiverObject(rDelegate.m_ReceiverObject)
      , m_ReceiverMethod(rDelegate.m_ReceiverMethod)
    {
    }

    /**
     * Destructor
     */
    virtual ~Delegate()
    {
    }

  public:
    void Notify(const void*, TArgs& rArguments)
    {
      (m_ReceiverObject->*m_ReceiverMethod)(rArguments);
    }

    AbstractDelegate<TArgs>* Clone() const
    {
      return new Delegate(*this);
    }

  public:
    /** 
     * Assignment operator. Assign another Delegate
     * @param rOther
     * @return reference to this FunctionDelegate
     */
    Delegate& operator=(const Delegate& rOther)
    {
      if (&rOther != this)
      {
        this->m_pTarget        = rOther.m_pTarget;
        this->m_ReceiverObject = rOther.m_ReceiverObject;
        this->m_ReceiverMethod = rOther.m_ReceiverMethod;
      }

      return *this;
    }

  protected:
    TObj*        m_ReceiverObject;
    NotifyMethod m_ReceiverMethod;

  private:
    Delegate();
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Wraps a C style function (or a C++ static function) to be used as
   * a delegate
   */
  template <class TArgs, bool hasSender = true, bool senderIsConst = true> 
  class FunctionDelegate: public AbstractDelegate<TArgs>
  {
  public:
    /**
     * Notify callback function
     */
    typedef void (*NotifyMethod)(const void*, TArgs&);

    /**
     * Creates a FunctionDelegate with the given NotifyMethod
     * @param method
     */
    FunctionDelegate(NotifyMethod method)
      : AbstractDelegate<TArgs>(*reinterpret_cast<void**>(&method))
      , m_ReceiverMethod(method)
    {
    }

    /**
     * Copy constructor. Creates the FunctionDelegate from another one.
     * @param rOther
     */
    FunctionDelegate(const FunctionDelegate& rOther)
      : AbstractDelegate<TArgs>(rOther)
      , m_ReceiverMethod(rOther.m_ReceiverMethod)
    {
    }

    /**
     * Destructor
     */
    virtual ~FunctionDelegate()
    {
    }

  public:
    void Notify(const void* pSender, TArgs& rArguments)
    {
      (*m_ReceiverMethod)(pSender, rArguments);
    }

    AbstractDelegate<TArgs>* Clone() const
    {
      return new FunctionDelegate(*this);
    }

  public:
    /** 
     * Assignment operator. Assign another FunctionDelegate
     * @param rOther
     * @return reference to this FunctionDelegate
     */
    FunctionDelegate& operator=(const FunctionDelegate& rOther)
    {
      if (&rOther != this)
      {
        this->m_pTarget        = rOther.m_pTarget;
        this->m_ReceiverMethod = rOther.m_ReceiverMethod;
      }
      return *this;
    }

  protected:
    /** 
     * Callback method
     */
    NotifyMethod m_ReceiverMethod;

  private:
    FunctionDelegate();
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Wraps a C style function (or a C++ static function) to be used as
   * a delegate
   */
  template <class TArgs> 
  class FunctionDelegate<TArgs, true, false>: public AbstractDelegate<TArgs>
  {
  public:
    /**
     * Notify callback function
     */
    typedef void (*NotifyMethod)(void*, TArgs&);

    /**
     * Creates a FunctionDelegate with the given NotifyMethod
     * @param method
     */
    FunctionDelegate(NotifyMethod method)
      : AbstractDelegate<TArgs>(*reinterpret_cast<void**>(&method))
      , m_ReceiverMethod(method)
    {
    }

    /**
     * Copy constructor. Creates the FunctionDelegate from another one.
     * @param rOther
     */
    FunctionDelegate(const FunctionDelegate& rOther)
      : AbstractDelegate<TArgs>(rOther)
      , m_ReceiverMethod(rOther.m_ReceiverMethod)
    {
    }

    /**
     * Destructor
     */
    virtual ~FunctionDelegate()
    {
    }

  public:
    void Notify(const void* pSender, TArgs& rArguments)
    {
      (*m_ReceiverMethod)(const_cast<void*>(pSender), rArguments);
    }

    AbstractDelegate<TArgs>* Clone() const
    {
      return new FunctionDelegate(*this);
    }

  public:
    /** 
     * Assignment operator. Assign another FunctionDelegate
     * @param rOther
     * @return reference to this FunctionDelegate
     */
    FunctionDelegate& operator=(const FunctionDelegate& rOther)
    {
      if (&rOther != this)
      {
        this->m_pTarget        = rOther.m_pTarget;
        this->m_ReceiverMethod = rOther.m_ReceiverMethod;
      }
      return *this;
    }

  protected:
    /** 
     * Callback method
     */
    NotifyMethod m_ReceiverMethod;

  private:
    FunctionDelegate();
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Wraps a C style function (or a C++ static function) to be used as
   * a delegate
   */
  template <class TArgs, bool senderIsConst> 
  class FunctionDelegate<TArgs, false, senderIsConst>: public AbstractDelegate<TArgs>
  {
  public:
    /**
     * Notify callback function
     */
    typedef void (*NotifyMethod)(TArgs&);

    /**
     * Creates a FunctionDelegate with the given NotifyMethod
     * @param method
     */
    FunctionDelegate(NotifyMethod method)
      : AbstractDelegate<TArgs>(*reinterpret_cast<void**>(&method))
      , m_ReceiverMethod(method)
    {
    }

    /**
     * Copy constructor. Creates the FunctionDelegate from another one.
     * @param rOther
     */
    FunctionDelegate(const FunctionDelegate& rOther)
      : AbstractDelegate<TArgs>(rOther)
      , m_ReceiverMethod(rOther.m_ReceiverMethod)
    {
    }

    /**
     * Destructor
     */
    virtual ~FunctionDelegate()
    {
    }

  public:
    void Notify(const void* /*pSender*/, TArgs& rArguments)
    {
      (*m_ReceiverMethod)(rArguments);
    }

    AbstractDelegate<TArgs>* Clone() const
    {
      return new FunctionDelegate(*this);
    }

  public:
    /** 
     * Assignment operator. Assign another FunctionDelegate
     * @param rOther
     * @return reference to this FunctionDelegate
     */
    FunctionDelegate& operator=(const FunctionDelegate& rOther)
    {
      if (&rOther != this)
      {
        this->m_pTarget        = rOther.m_pTarget;
        this->m_ReceiverMethod = rOther.m_ReceiverMethod;
      }
      return *this;
    }

  protected:
    /** 
     * Callback method
     */
    NotifyMethod m_ReceiverMethod;

  private:
    FunctionDelegate();
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The interface that all notification strategies must implement.
   * 
   * Note: Event is based on policy-driven design, so your strategy implementation
   * must offer all the methods from this interface (otherwise: compile errors)
   * but you don't need to inherit from NotificationStrategy.
   */
  template <class TArgs> 
  class NotificationStrategy
  {
  public:
    /** 
     * Default constructor
     */
    NotificationStrategy()
    {
    }

    /**
     * Destructor
     */
    virtual ~NotificationStrategy()
    {
    }

  public:
    /**
     * Sends a notification to all registered delegates,
     */
    virtual void Notify(const void* sender, TArgs& arguments) = 0;

    /**
     * Adds a delegate to the strategy, if the delegate is not yet present
     */
    virtual void Add(const AbstractDelegate<TArgs>& pDelegate) = 0;

    /**
     * Removes a delegate from the strategy if found.
     */
    virtual void Remove(const AbstractDelegate<TArgs>& pDelegate) = 0;

    /**
     * Removes all delegates from the strategy.
     */
    virtual void Clear() = 0;

    /**
     * Returns false if the strategy contains at least one delegate.
     */
    virtual bool IsEmpty() const = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Default notification strategy. Allows one observer
   * to register exactly once. The observer must provide an
   * < (less-than) operator.
   */
  template <class TArgs> 
  class DefaultStrategy : public NotificationStrategy<TArgs>
  {
  public:
    /**
     * Type declaration of a DelegateList 
     */
    typedef karto::List< AbstractDelegate<TArgs>* > DelegateList;

  public:
    /**
     * Default constructor
     */
    DefaultStrategy()
    {
    }

    /**
     * Copy constructor. Creates the DefaultStrategy from another one.
     * @param rOther
     */
    DefaultStrategy(const DefaultStrategy& rOther)
    {
      operator = (rOther);
    }

    /**
     * Destructor
     */
    virtual ~DefaultStrategy()
    {
      Clear();
    }

  public:
    void Notify(const void* pSender, TArgs& rArguments)
    {
      karto_forEach(typename DelegateList, &m_Observers)
      {
        (*iter)->Notify(pSender, rArguments);
      }
    }

    void Add(const AbstractDelegate<TArgs>& rDelegate)
    {
      Remove(rDelegate);

      AbstractDelegate<TArgs>* pDelegate = rDelegate.Clone();
      m_Observers.Add(pDelegate);
    }

    void Remove(const AbstractDelegate<TArgs>& rDelegate)
    {
      kt_bool found = false;
      kt_int32s index = 0;
      karto_forEach(typename DelegateList, &m_Observers)
      {
        if (*(*iter) == rDelegate)
        {
          delete *iter;
          found = true;
          break;
        }

        index++;
      }

      if (found == true)
      {
        m_Observers.RemoveAt(index);
      }
    }

    void Clear()
    {
      karto_forEach(typename DelegateList, &m_Observers)
      {
        delete *iter;
      }
      m_Observers.Clear();
    }

    kt_bool IsEmpty() const
    {
      return m_Observers.IsEmpty();
    }

  public:
    /**
     * Assignment operator
     */
    DefaultStrategy& operator=(const DefaultStrategy& rOther)
    {
      if (this != &rOther)
      {
        karto_const_forEach(typename DelegateList, &(rOther.m_Observers))
        {
          Add(**iter);
        }
      }
      return *this;
    }

  protected:
    /**
     * Delegate observer list
     */
    DelegateList m_Observers;
  };
  
  // @endcond

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * An AbstractEvent is the super-class of all events. 
   * It works similar to the way C# handles notifications (aka events in C#).
   * Events can be used to send information to a set of observers
   * which are registered at the event. The type of the data is specified with
   * the template parameter TArgs. 
   *
   * \code
   *     #include <Karto/Event.h>
   * \endcode
   * Use events by adding them as public members to the object which is throwing notifications:
   * \code
   *     class MyData
   *     {
   *     public:
   *         karto::BasicEvent<int> AgeChanged;
   *         
   *         MyData();
   *         ...
   *     };
   * \endcode
   * Throwing the event can be done either by the events Notify() method:
   *
   * Alternatively, instead of Notify(), operator() can be used.
   * \code
   *     void MyData::setAge(int i)
   *     {
   *         this->_age = i;
   *         AgeChanged(this, this->_age);
   *     }
   * \endcode
   * Note that Notify() do not catch exceptions, i.e. in case a delegate 
   * throws an exception, the notify is immediately aborted and the exception is thrown
   * back to the caller.
   *
   * Delegates can register methods at the event. In the case of a BasicEvent
   * the Delegate template is used.
   *
   * Events require the observers to follow one of the following method signature:
   * \code
   *     void OnEvent(const void* pSender, TArgs& args);
   *     void OnEvent(TArgs& args);
   *     static void OnEvent(const void* pSender, TArgs& args);
   *     static void OnEvent(void* pSender, TArgs& args);
   *     static void OnEvent(TArgs& args);
   * \endcode
   * For performance reasons arguments are always sent by reference. This also allows observers
   * to modify the sent argument. To prevent that, use \<const TArg\> as template
   * parameter. A non-conformant method signature leads to compile errors.
   *
   * Assuming that the observer meets the method signature requirement, it can register
   * this method with the += operator:
   * \code
   *     class MyController
   *     {
   *     protected:
   *         MyData _data;
   *         
   *         void onDataChanged(void* pSender, int& data);
   *         ...
   *     };
   *         
   *     MyController::MyController()
   *     {
   *         _data.AgeChanged += karto::delegate(this, &MyController::onDataChanged);
   *     }
   * \endcode
   *
   * Unregistering happens via the -= operator. Forgetting to unregister a method will lead to
   * segmentation faults later, when one tries to send a notify to a no longer existing object.
   * \code
   *     MyController::~MyController()
   *     {
   *         _data.DataChanged -= karto::delegate(this, &MyController::onDataChanged);
   *     }
   * \endcode
   */
  template <class TArgs> 
  class AbstractEvent
  {
  public:
    /**
     * Default constructor
     */
    AbstractEvent()
      : m_Enabled(true)
    {
    }

     /**
     * Constructs an event with a default strategy 
     * @param rStrategy
     */
    AbstractEvent(const DefaultStrategy<TArgs>& rStrategy)
      : m_Enabled(true)
      , m_Strategy(rStrategy)
    {
    }

    /**
     * Destructor
     */
    virtual ~AbstractEvent()
    {
    }

    /**
     * Adds a delegate to the event. If the observer is equal to an
     * already existing one (determined by the < operator),
     * it will simply replace the existing observer.
     * This behavior is determined by the TStrategy. Current implementations
     * (DefaultStrategy) follow that guideline but future ones
     * can deviate.
     * @param rDelegate
     */
    void operator+=(const AbstractDelegate<TArgs>& rDelegate)
    {
      Mutex::ScopedLock lock(m_Mutex);
      m_Strategy.Add(rDelegate);
    }

    /**
     * Removes a delegate from the event. If the delegate is equal to an
     * already existing one is determined by the < operator.
     * If the observer is not found, the unregister will be ignored
     * @param rDelegate
     */
    void operator-=(const AbstractDelegate<TArgs>& rDelegate)
    {
      Mutex::ScopedLock lock(m_Mutex);
      m_Strategy.Remove(rDelegate);
    }

    /**
     * Same as Notify
     */
    void operator()(const void* pSender, TArgs& args)
    {
      Notify(pSender, args);
    }

    /**
     * Sends a notification to all registered delegates. The order is 
     * determined by the TStrategy. This method is blocking. While executing,
     * other objects can change the list of delegates. These changes don't
     * influence the current active notifications but are activated with
     * the next notify. If one of the delegates throws an exception, the notify
     * method is immediately aborted and the exception is reported to the caller.
     * @param pSender
     * @param rArgs
     */
    void Notify(const void* pSender, TArgs& rArgs)
    {
      DefaultStrategy<TArgs>* pStrats = NULL;
      kt_bool enabled = false;

      {
        Mutex::ScopedLock lock(m_Mutex);
        enabled = m_Enabled;

        if (m_Enabled)
        {
          // thread-safeness: 
          // copy should be faster and safer than blocking until execution ends
          pStrats = new DefaultStrategy<TArgs>(m_Strategy);
        }
      }

      if (enabled)
      {
        pStrats->Notify(pSender, rArgs);
      }

      delete pStrats;
    }

    /**
     * Enables the event.
     */
    void Enable()
    {
      Mutex::ScopedLock lock(m_Mutex);
      m_Enabled = true;
    }

    /**
     * Disables the event. Notify will be ignored,
     * but adding/removing delegates is still allowed.
     */
    void Disable()
    {
      Mutex::ScopedLock lock(m_Mutex);
      m_Enabled = false;
    }

    /**
     * Checks if we are enabled
     */
    kt_bool IsEnabled() const
    {
      Mutex::ScopedLock lock(m_Mutex);
      return m_Enabled;
    }

    /**
     * Removes all delegates.
     */
    void Clear()
    {
      Mutex::ScopedLock lock(m_Mutex);
      m_Strategy.Clear();
    }

    /**
     * Checks if any delegates are registered at the delegate.
     */
    kt_bool IsEmpty() const
    {
      Mutex::ScopedLock lock(m_Mutex);
      return m_Strategy.IsEmpty();
    }

  protected:
    /**
     * Enabled flag
     */
    kt_bool m_Enabled; 

    /**
     * Strategy
     */
    DefaultStrategy<TArgs> m_Strategy; 

    /**
     * Mutex
     */
    mutable Mutex m_Mutex;

  private:
    AbstractEvent(const AbstractEvent& rOther);
    AbstractEvent& operator=(const AbstractEvent& rOther);
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * A BasicEvent uses internally a DefaultStrategy which 
   * invokes delegates in an arbitrary manner.
   * Note that an object can only register one method to a BasicEvent.
   * Subsequent registrations will overwrite the existing delegate.
   * \code
   *     BasicEvent<int> event;
   *     MyClass myObject;
   *     event += karto::delegate(&myObject, &MyClass::myMethod1);
   *     event += karto::delegate(&myObject, &MyClass::myMethod2);
   * \endcode
   * The second registration will overwrite the first one. The reason is simply that
   * function pointers can only be compared by equality but not by lower than.
   */
  template <class TArgs> 
  class BasicEvent : public AbstractEvent <TArgs >
  {
  public:
    BasicEvent()
    {
    }

    virtual ~BasicEvent()
    {
    }

  private:
    BasicEvent(const BasicEvent& e);
    BasicEvent& operator=(const BasicEvent& e);
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The purpose of the EventArguments class is to be used as parameter
   * when one doesn't want to send any data.
   * One can use EventArgs as a super-class for one's own event arguments
   * but with the arguments being a template parameter this is not
   * necessary.
   */
  class KARTO_EXPORT EventArguments
  {
  public:
    EventArguments();
    virtual ~EventArguments();

  public:
    /**
     * An empty event argument
     * @return empty event argument
     */
    static EventArguments& Empty()
    {
      static EventArguments dummy;

      return dummy;
    }
  }; // class EventArguments

#ifdef WIN32
#define EXPORT_KARTO_EVENT(declspec, T) \
  template class declspec karto::BasicEvent<T>;

  EXPORT_KARTO_EVENT(KARTO_EXPORT, EventArguments)
#endif

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template <class TObj, class TArgs>
  static Delegate<TObj, TArgs, true> delegate(TObj* pObj, void (TObj::*NotifyMethod)(const void*, TArgs&))
  {
    return Delegate<TObj, TArgs, true>(pObj, NotifyMethod);
  }

  template <class TObj, class TArgs>
  static Delegate<TObj, TArgs, false> delegate(TObj* pObj, void (TObj::*NotifyMethod)(TArgs&))
  {
    return Delegate<TObj, TArgs, false>(pObj, NotifyMethod);
  }

  template <class TArgs>
  static FunctionDelegate<TArgs, true, true> delegate(void (*NotifyMethod)(const void*, TArgs&))
  {
    return FunctionDelegate<TArgs, true, true>(NotifyMethod);
  }

  template <class TArgs>
  static FunctionDelegate<TArgs, true, false> delegate(void (*NotifyMethod)(void*, TArgs&))
  {
    return FunctionDelegate<TArgs, true, false>(NotifyMethod);
  }

  template <class TArgs>
  static FunctionDelegate<TArgs, false> delegate(void (*NotifyMethod)(TArgs&))
  {
    return FunctionDelegate<TArgs, false>(NotifyMethod);
  }

  //@}

}

#endif // __OpenKarto_Event_h__