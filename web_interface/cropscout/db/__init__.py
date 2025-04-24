from sqlmodel import Session, SQLModel, create_engine

__all__ = ['Session', 'get_session', 'setup_db']

DATABASE_URL = 'sqlite:///cropscout.db'
engine = create_engine(DATABASE_URL)


def get_session() -> None:
    with Session(engine) as session:
        yield session


def setup_db() -> None:
    SQLModel.metadata.drop_all(engine)  # for development stage
    SQLModel.metadata.create_all(engine)
